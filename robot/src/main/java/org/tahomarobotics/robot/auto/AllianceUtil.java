package org.tahomarobotics.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class AllianceUtil {

    public static final double FIELD_LENGTH = 651.25;  // inches

    /**
     * Translates a field pose from a blue alliance path to the equivalent pose
     * for the red alliance.  If on blue alliance no transformation takes place.
     * @param pose the pose from auto path
     * @return the pose adjusted for current alliance
     */
    public static Pose2d adjustPoseForAlliance(Pose2d pose) {
        if (DriverStation.getAlliance() != DriverStation.Alliance.Blue) {
            double x = Math.abs(FIELD_LENGTH - pose.getX());
            double radians = pose.getRotation().getRadians();
            Rotation2d theta = new Rotation2d(-Math.cos(radians), Math.sin(radians));
            pose = new Pose2d(x, pose.getY(), theta);
        }
        return pose;
    }
}
