package org.tahomarobotics.robot.vision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.EnumSet;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

public class Vision {
    private static final Logger logger = LoggerFactory.getLogger(Vision.class);

    public interface VisionListener {
        void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds);
    }

    /**
     * Helper method to convert measured target data to 3D pose units
     */
    private static Pose3d createTarget(double x, double y, double z, double roll, double pitch, double yaw) {
        return new Pose3d(
                new Translation3d(
                        Units.inchesToMeters(x),
                        Units.inchesToMeters(y),
                        Units.inchesToMeters(z)
                ),
                new Rotation3d(
                        Units.degreesToRadians(roll),
                        Units.degreesToRadians(pitch),
                        Units.degreesToRadians(yaw)
                )
        );
    }

    // collection of field targets
    private static final Map<Integer, Pose3d> TARGETS = new HashMap<>();

//    COMP
//    static {
//        TARGETS.put(1, createTarget(610.77, 42.19, 18.22, 0, 0, 180));
//        TARGETS.put(2, createTarget(610.77, 108.19, 18.22, 0, 0, 180));
//        TARGETS.put(3, createTarget(610.77, 147.19, 18.22, 0, 0, 180));
//        TARGETS.put(4, createTarget(636.96, 265.74, 27.38, 0, 0, 180));
//        TARGETS.put(5, createTarget(14.25, 265.74, 27.38, 0, 0, 0));
//        TARGETS.put(6, createTarget(40.45, 147.19, 18.22, 0, 0, 0));
//        TARGETS.put(7, createTarget(40.45, 108.19, 18.22, 0, 0, 0));
//        TARGETS.put(8, createTarget(40.45, 42.19, 18.22, 0, 0, 0));
//    }

    static {
        TARGETS.put(0, createTarget(Units.metersToInches(2.67), 0, Units.metersToInches(.56), 0, 0, 90));
        TARGETS.put(4, createTarget(Units.metersToInches(4.67), Units.metersToInches(-.045), Units.metersToInches(.55), 0, 0, 90));
        TARGETS.put(1, createTarget(Units.metersToInches(3.875), Units.metersToInches(5.205), Units.metersToInches(.535), 0, 0, 270));
        TARGETS.put(2, createTarget(Units.metersToInches(6.025 - 1.10), Units.metersToInches(5.205 - .30), Units.metersToInches(.53), 0, 0, 270));
    }


    private static final Transform3d cameraToCenter = new Transform3d(
            new Translation3d(
                    -VisionConstants.CAMERA_TO_CENTER,
                    0,
                    VisionConstants.CAMERA_HEIGHT
            ),
            new Rotation3d(
                    Units.degreesToRadians(0),
                    -VisionConstants.CAMERA_ANGLE,
                    Units.degreesToRadians(0)
            )).inverse();

    private final PhotonCamera camera;

    private final VisionListener listener;

    private final DoubleSubscriber latencySub;

    public Vision(VisionListener listener) {

        this.listener = listener;

        // normally this would the default client connecting to robot
        // connect to server running on camera (for debugging0
        NetworkTableInstance inst = NetworkTableInstance.getDefault();

        // create PhotonLib camera (required for each camera)
        camera = new PhotonCamera(inst, "OV5647");

        // subscribe to new data from photonvision
        latencySub = inst.getTable("photonvision").getSubTable("OV5647").getDoubleTopic("latencyMillis").subscribe(0.0);
        inst.addListener(
                latencySub,
                EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                e -> {
                    processVisionUpdate(camera.getLatestResult());
                }
        );
    }

    /**
     * process new camera data from a subscribed camera
     *
     * @param result - new data from a camera
     */
    private void processVisionUpdate(PhotonPipelineResult result) {

        double time = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - result.getLatencyMillis() * 1.0e-3;

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

            Pose3d cameraPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), targetPose, cameraToCenter);
            // extract the transform (translation and rotation) from camera to target
//            Transform3d targetTransform = target.getBestCameraToTarget();
//
//            // subtract the transform from the target to obtain the camera position
//            Pose3d cameraPose = targetPose.transformBy(targetTransform.inverse());
//
//            cameraPose = cameraPose.transformBy(cameraToCenter);

//            System.out.format("ID: %d - (x: %.3f - y: %.3f - z: %.3f) - (roll: %.3f - pitch: %.3f - yaw: %.3f\n",
//                    targetId, cameraPose.getX(), cameraPose.getY(), cameraPose.getZ(),
//                    Units.radiansToDegrees(cameraPose.getRotation().getX()),
//                    Units.radiansToDegrees(cameraPose.getRotation().getY()),
//                    Units.radiansToDegrees(cameraPose.getRotation().getZ()));

            // call back to listener
            try {
                listener.addVisionMeasurement(cameraPose.toPose2d(), time);
            } catch (Exception ignored) {
//                DriverStation.reportError(String.format("ERROR at pose: " + cameraPose + " with latency " + result.getLatencyMillis() + " with error: " + e), false);
            }
        }
    }

}
