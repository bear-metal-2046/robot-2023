package org.tahomarobotics.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.List;
import java.util.function.Function;
import java.util.stream.Collectors;

public class AllianceUtil {

    public static final double FIELD_LENGTH = Units.inchesToMeters(651.25);

    /**
     * Translates a field pose from a blue alliance path to the equivalent pose
     * for the red alliance.  If on blue alliance no transformation takes place.
     *
     * @param pose the pose from auto path
     * @return the pose adjusted for current alliance
     */
    public static Pose2d adjustPoseForAlliance(Pose2d pose) {
        return adjustPoseForAlliance(DriverStation.getAlliance(), pose);
    }

    public static Pose2d adjustPoseForAlliance(DriverStation.Alliance alliance, Pose2d pose) {
        if (alliance != DriverStation.Alliance.Blue) {
            double x = Math.abs(FIELD_LENGTH - pose.getX());
            double radians = pose.getRotation().getRadians();
            Rotation2d theta = new Rotation2d(-Math.cos(radians), Math.sin(radians));
            pose = new Pose2d(x, pose.getY(), theta);
        }
        return pose;
    }

    static Function<Translation2d, Translation2d> RED_ORIGIN = (Translation2d t) -> new Translation2d(FIELD_LENGTH - t.getX(), t.getY());
    private static final Rotation2d RED_ROTATION = new Rotation2d(Math.PI);

    static Function<Pose2d, Translation2d> RED_TRANSLATE = (Pose2d p) -> new Translation2d(FIELD_LENGTH - p.getX(), p.getY());
    public static List<Translation2d>  convertToRed(List<Translation2d> interiorWaypoints) {
        return interiorWaypoints.stream().map(w -> RED_ORIGIN.apply(w))
                .collect(Collectors.toList());
    }

    public static Translation2d  convertToRed(Translation2d xy) {
        return RED_ORIGIN.apply(xy);
    }

    public static Pose2d  convertToRed(Pose2d pose) {
        return new Pose2d(
                RED_TRANSLATE.apply(pose),
                RED_ROTATION.minus(pose.getRotation()));
    }

    public static Rotation2d adjustAngleForAlliance(DriverStation.Alliance alliance, Rotation2d angle) {
        if (alliance == DriverStation.Alliance.Red) {
            angle = RED_ROTATION.minus(angle);
        }
        return angle;
    }

    public record TrajectoryPair(Trajectory blueSide, Trajectory redSide) {
        public Trajectory getTrajectory() {
            return DriverStation.getAlliance() == DriverStation.Alliance.Blue ? blueSide : redSide;
        }
    }
    public static TrajectoryPair createTrajectoryPair(Pose2d start, List<Translation2d> interiorWaypoints, Pose2d end, TrajectoryConfig config) {
        return new TrajectoryPair(
                TrajectoryGenerator.generateTrajectory( start, interiorWaypoints, end, config ),
                TrajectoryGenerator.generateTrajectory(
                        convertToRed(start),
                        convertToRed(interiorWaypoints),
                        convertToRed(end),
                        config));
    }

    public static TrajectoryPair createTrajectoryPair(Pose2d start, Pose2d end, TrajectoryConfig config) {
        return createTrajectoryPair(start, List.of(), end, config);
    }

    public static Trajectory createTrajectory(DriverStation.Alliance alliance, Pose2d start, Pose2d end, TrajectoryConfig config) {
        return createTrajectory(alliance, start, List.of(), end, config);
    }
    public static Trajectory createTrajectory(DriverStation.Alliance alliance, Pose2d start, List<Translation2d> interiorWaypoints, Pose2d end, TrajectoryConfig config) {
        return alliance == DriverStation.Alliance.Blue ?

                // blue has no conversion
                TrajectoryGenerator.generateTrajectory( start, interiorWaypoints, end, config) :

                // red needs to be converted
                TrajectoryGenerator.generateTrajectory(
                        convertToRed(start),
                        convertToRed(interiorWaypoints),
                        convertToRed(end),
                        config);
    }
}
