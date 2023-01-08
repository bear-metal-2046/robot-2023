package org.tahomarobotics.robot.Vision.AprilTags;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.WPIUtilJNI;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.EnumSet;
import java.util.HashMap;
import java.util.Map;

public class AprilTagVision {

    @FunctionalInterface
    public interface VisionListener {
        void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds);
    }

    /**
     * Helper method to convert measured target data to 3D pose units
     */
    private static Pose3d createTarget(double x, double y, double z, double yaw) {
        return new Pose3d(
                new Translation3d(
                        Units.inchesToMeters(x),
                        Units.inchesToMeters(y),
                        Units.inchesToMeters(z)
                ),
                new Rotation3d(
                        Units.degreesToRadians(0),
                        Units.degreesToRadians(0),
                        Units.degreesToRadians(yaw)
                )
        );
    }

    // collection of field targets
    private static final Map<Integer, Pose3d> TARGETS = new HashMap<>();

    static {
        TARGETS.put(1, createTarget(610.77, 42.19, 18.22,  180));
        TARGETS.put(2, createTarget(610.77, 108.19, 18.22,  180));
        TARGETS.put(3, createTarget(610.77, 147.19, 18.22,  180));
        TARGETS.put(4, createTarget(636.96, 265.74, 27.38,  180));
        TARGETS.put(5, createTarget(14.25, 265.74, 27.38,  0));
        TARGETS.put(6, createTarget(40.45, 147.19, 18.22,  0));
        TARGETS.put(7, createTarget(40.45, 108.19, 18.22,  0));
        TARGETS.put(8, createTarget(40.45, 42.19, 18.22,  0));
    }

    private static final Transform3d cameraToCenter = new Transform3d(
            new Translation3d(
                    -AprilTagVisionConstants.CAMERA_TO_CENTER,
                    0,
                    AprilTagVisionConstants.CAMERA_HEIGHT
            ),
            new Rotation3d(
                    Units.degreesToRadians(0),
                    -AprilTagVisionConstants.CAMERA_ANGLE,
                    Units.degreesToRadians(180)
            )).inverse();

    private final PhotonCamera camera;

    private final VisionListener listener;

    private final DoubleSubscriber latencySub;
    private final int latencySubHandler;

    private boolean disabled;

    public AprilTagVision(VisionListener listener) {
        disabled = false;

        this.listener = listener;

        // normally this would the default client connecting to robot
        // connect to server running on camera (for debugging)
        NetworkTableInstance inst = NetworkTableInstance.getDefault();

        // create PhotonLib camera (required for each camera)
        camera = new PhotonCamera(inst, "photonvision");

        NetworkTable photonvision = inst.getTable("photonvision").getSubTable("photonvision");

        latencySub = photonvision.getDoubleTopic("latencyMillis").subscribe(0.0);

        latencySubHandler = inst.addListener(
            latencySub,
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            event -> processVisionUpdate(camera.getLatestResult())
        );
    }

    /**
     * process new camera data from a subscribed camera
     *
     * @param result - new data from a camera
     */
    private void processVisionUpdate(PhotonPipelineResult result) {

        double time = WPIUtilJNI.now() * 1.0e-6 - result.getLatencyMillis() * 1.0e-3;

        // loop through each target
        for (PhotonTrackedTarget target : result.targets) {

            if (target.getPoseAmbiguity() > 0.2) continue;

            // get 3D pose information for identified target (aprilTag)
            int targetId = target.getFiducialId();
            Pose3d targetPose = TARGETS.get(targetId);

            // skip if aprilTag not recognized
            if (targetPose == null) {
                continue;
            }

            // extract the transform (translation and rotation) from camera to target
            Transform3d targetTransform = target.getBestCameraToTarget();

            // subtract the transform from the target to obtain the camera position
            Pose3d cameraPose = targetPose.transformBy(targetTransform.inverse());

            cameraPose = cameraPose.transformBy(cameraToCenter);

//            System.out.format("ID: %d - (x: %.3f - y: %.3f - z: %.3f) - (roll: %.3f - pitch: %.3f - yaw: %.3f\n",
//                    targetId, Units.metersToInches(cameraPose.getX()), Units.metersToInches(cameraPose.getY()), Units.metersToInches(cameraPose.getZ()),
//                    Units.radiansToDegrees(cameraPose.getRotation().getX()),
//                    Units.radiansToDegrees(cameraPose.getRotation().getY()),
//                    Units.radiansToDegrees(cameraPose.getRotation().getZ()));

            // call back to listener
            listener.addVisionMeasurement(cameraPose.toPose2d(), time);
        }
    }

    public void close() {
        if (!disabled) {
            NetworkTableInstance inst = NetworkTableInstance.getDefault();
            inst.removeListener(latencySubHandler);
            latencySub.close();

            disabled = true;
        }
    }
}
