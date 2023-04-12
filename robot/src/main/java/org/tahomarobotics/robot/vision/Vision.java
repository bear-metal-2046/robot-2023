package org.tahomarobotics.robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.IOException;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import java.util.Optional;

public class Vision {
    private static final Logger logger = LoggerFactory.getLogger(Vision.class);

    public record PVCameraResult(PVCamera camera, double timestamp, Pose2d poseMeters, double distanceToTargets, int numTargets) {}

    public interface VisionListener {
        void addVisionMeasurement(PVCameraResult result);
    }

    public enum PVCamera {
        LEFT_FRONT(VisionConstants.LEFT_FRONT_CAM_OFFSET, "Left Front"),
        RIGHT_FRONT(VisionConstants.RIGHT_FRONT_CAM_OFFSET, "Right Front"),
        BACK(VisionConstants.BACK_CAM_OFFSET, "Back");

        public final Transform3d offset;
        public final String cameraName;

        PVCamera(Transform3d offset, String cameraName) {
            this.offset = offset;
            this.cameraName = cameraName;
        }
    }

    private final PhotonCamera camera;

    private final VisionListener listener;

    private AprilTagFieldLayout fieldLayout = null;
    private PhotonPoseEstimator photonPoseEstimator = null;
    private final PVCamera cameraSettings;

    public Vision(VisionListener listener, PVCamera cameraSettings) {
        this.cameraSettings = cameraSettings;
        this.listener = listener;

        // normally this would the default client connecting to robot
        // connect to server running on camera (for debugging0
        NetworkTableInstance inst = NetworkTableInstance.getDefault();

        // create PhotonLib camera (required for each camera)
        camera = new PhotonCamera(inst, cameraSettings.cameraName);

        try {
            fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();

            // Photon pose estimator will use all seen tags to run PnP once,
            // instead of looking at each target indiviudally
            photonPoseEstimator = new PhotonPoseEstimator(
                    fieldLayout,
                    PoseStrategy.MULTI_TAG_PNP,
                    camera,
                    cameraSettings.offset
            );
        } catch (IOException e) {
            logger.error("Unable to load field layout!");
            logger.error(e.toString());
        }

        // subscribe to new data from photonvision
        DoubleSubscriber latencySub = inst.getTable("photonvision").getSubTable(cameraSettings.cameraName).getDoubleTopic("latencyMillis").subscribe(0.0);
        inst.addListener(
            latencySub,
                EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                e -> processVisionUpdate(camera.getLatestResult())
        );
    }

    private void processSingleTarget(PhotonTrackedTarget target, double timestampSeconds) {
        Optional<Pose3d> tagPose = fieldLayout.getTagPose(target.getFiducialId());
        Transform3d bestCamPose = target.getBestCameraToTarget();

        if (tagPose.isEmpty() || target.getPoseAmbiguity() > 0.2) {
            return;
        }

        double distance = Math.hypot(bestCamPose.getX(), bestCamPose.getY());
        Pose3d newRobotPose = PhotonUtils.estimateFieldToRobotAprilTag(
            bestCamPose,
            tagPose.get(),
            cameraSettings.offset.inverse()
        );

        listener.addVisionMeasurement(new PVCameraResult(
            cameraSettings,
            timestampSeconds, // Photonvision timestamp
            newRobotPose.toPose2d(),
            distance,
            1
        ));
    }

    /**
     * process new camera data from a subscribed camera
     *
     * @param result - new data from a camera
     */
    private void processVisionUpdate(PhotonPipelineResult result) {
        List<PhotonTrackedTarget> validTargets = new ArrayList<>();

        // Limelight can sometimes return tags that do not exist. Filter these out so that we have an accurate count.
        // This also fixes PhotonPoseEstimator using corners from bad results when doing PnP :(
        for (PhotonTrackedTarget target : result.targets) {
            Optional<Pose3d> tagPose = fieldLayout.getTagPose(target.getFiducialId());
            if (tagPose.isPresent()) {
                validTargets.add(target);
            }
        }

        if (validTargets.size() > 1 && photonPoseEstimator != null) {
            // Multi-tag PnP if more than one target is found
            PhotonPipelineResult correctedResult = new PhotonPipelineResult(result.getLatencyMillis(), validTargets);
            correctedResult.setTimestampSeconds(result.getTimestampSeconds());
            Optional<EstimatedRobotPose> position = photonPoseEstimator.update(correctedResult);

            if (position.isEmpty()) return;

            double distances = 0;
            for (PhotonTrackedTarget target : position.get().targetsUsed) {
                Transform3d pose = target.getBestCameraToTarget();
                distances = Math.max(distances, Math.hypot(pose.getX(), pose.getY()));
            }

            listener.addVisionMeasurement(new PVCameraResult(
                cameraSettings,
                position.get().timestampSeconds, // Photonvision timestamp
                position.get().estimatedPose.toPose2d(),
                distances,
                position.get().targetsUsed.size()
            ));
        } else if (validTargets.size() == 1 && fieldLayout != null) {
            processSingleTarget(validTargets.get(0), result.getTimestampSeconds());
        }
    }
}
