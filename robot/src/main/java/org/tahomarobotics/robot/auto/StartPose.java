package org.tahomarobotics.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import org.tahomarobotics.robot.Robot;

public enum StartPose {
    LEFT(Units.inchesToMeters(72.6), Units.inchesToMeters(34.5), Units.inchesToMeters(585.4), Units.inchesToMeters(175.2), Units.degreesToRadians(getTeamColorHdg())),
    MID(Units.inchesToMeters(72.3), Units.inchesToMeters(109.8), Units.inchesToMeters(585.3), Units.inchesToMeters(110.4), Units.degreesToRadians(getTeamColorHdg())),
    RIGHT(Units.inchesToMeters(72.9), Units.inchesToMeters(177.2), Units.inchesToMeters(585.4), Units.inchesToMeters(47), Units.degreesToRadians(getTeamColorHdg()));

    public static double getTeamColorHdg() {
        if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
            return 180;
        } else {
            return 0;
        }
    }

    private final Pose2d bluePose;
    private final Pose2d redPose;

    StartPose(double blueX, double blueY, double redX, double redY, double hdg) {
        this.bluePose = new Pose2d(blueX, blueY, new Rotation2d(hdg));
        this.redPose = new Pose2d(redX, redY, new Rotation2d(hdg));
    }
    //Sets the Starting pose of the robot at the beginning auto

    public static StartPose fromPose(Pose2d pose) {
        double poseX = pose.getX();
        double poseY = pose.getY();

        StartPose closestPose = StartPose.LEFT;
        double minDiff = Double.MAX_VALUE;
        for (StartPose enumPose : StartPose.values()) {
            Pose2d _pose = enumPose.getPose(
                    Robot.isReal() ? DriverStation.getAlliance() : DriverStation.Alliance.Blue
            );
            double deltaPoseX = poseX - _pose.getX();
            double deltaPoseY = poseY - _pose.getY();

            double diff = Math.sqrt((deltaPoseX * deltaPoseX) + (deltaPoseY * deltaPoseY));

            if (diff < minDiff) {
                minDiff = diff;
                closestPose = enumPose;
            }
        }

        return closestPose;
    }

    public Pose2d getPose() {
        if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
            return new Pose2d(bluePose.getX(), bluePose.getY(), bluePose.getRotation());
        } else {
            return new Pose2d(redPose.getX(), redPose.getY(), redPose.getRotation());
        }
    }
    //Get the pose when called of the robot

    public Pose2d getPose(DriverStation.Alliance alliance) {
        if (alliance == DriverStation.Alliance.Blue) {
            return new Pose2d(bluePose.getX(), bluePose.getY(), bluePose.getRotation());
        } else {
            return new Pose2d(redPose.getX(), redPose.getY(), redPose.getRotation());
        }
    }

    @Override
    public String toString() {
        return "StartPose{" + name() + " " +
                "pose=" + bluePose +
                '}';
    }
    //Gets the pose as a string
}
