package org.tahomarobotics.robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.photonvision.*;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.IOException;
import java.util.EnumSet;
import java.util.Optional;

public class Vision {
    private static final Logger logger = LoggerFactory.getLogger(Vision.class);

    public interface VisionListener {
        void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds, double distanceToTargets);
    }

    private static final Transform3d frontCameraOffset = new Transform3d(
            new Translation3d(
                    Units.inchesToMeters(-3.514), Units.inchesToMeters(-9.638), Units.inchesToMeters(23.616)
            ),
            new Rotation3d(
                    0, 0, Units.degreesToRadians(-15)
            )
    );

    private static final Transform3d backCameraOffset = new Transform3d(
            new Translation3d(
                    Units.inchesToMeters(-13.514), Units.inchesToMeters(-6.545), Units.inchesToMeters(23.616)
            ),
            new Rotation3d(
                    0, 0, Units.degreesToRadians(180)
            )
    );

    public enum PVCamera {
        FRONT(frontCameraOffset, "Front"),
        BACK(backCameraOffset, "Back");

        public final Transform3d offset;
        public final String cameraName;

        PVCamera(Transform3d offset, String cameraName) {
            this.offset = offset;
            this.cameraName = cameraName;
        }
    }

    private final PhotonCamera camera;

    private final VisionListener listener;

    private final DoubleSubscriber latencySub;

    private PhotonPoseEstimator photonPoseEstimator = null;

    public Vision(VisionListener listener, PVCamera cameraSettings) {

        this.listener = listener;

        // normally this would the default client connecting to robot
        // connect to server running on camera (for debugging0
        NetworkTableInstance inst = NetworkTableInstance.getDefault();

        // create PhotonLib camera (required for each camera)
        camera = new PhotonCamera(inst, cameraSettings.cameraName);

        try {
            AprilTagFieldLayout fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();

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
        latencySub = inst.getTable("photonvision").getSubTable(cameraSettings.cameraName).getDoubleTopic("latencyMillis").subscribe(0.0);
        inst.addListener(
                latencySub,
                EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                e -> processVisionUpdate(camera.getLatestResult())
        );
    }

    /**
     * process new camera data from a subscribed camera
     *
     * @param result - new data from a camera
     */
    private void processVisionUpdate(PhotonPipelineResult result) {
        if (result.targets.size() > 1 && photonPoseEstimator != null) {
            double distances = 0;
            for (PhotonTrackedTarget target : result.targets) {
                Transform3d pose = target.getBestCameraToTarget();
                distances += Math.hypot(pose.getX(), pose.getY());
            }

            // Multi-tag PnP if more than one target is found
            Optional<EstimatedRobotPose> position = photonPoseEstimator.update(result);
            if (position.isEmpty()) return;
            listener.addVisionMeasurement(position.get().estimatedPose.toPose2d(), position.get().timestampSeconds, distances / result.targets.size());
        }
    }
}
