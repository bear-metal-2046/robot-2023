package org.tahomarobotics.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;
import java.util.stream.Collectors;

public abstract class AutonomousBase extends SequentialCommandGroup implements AutonomousCommandIF{

    protected final List<Trajectory> trajectories = new ArrayList<>();

    protected final Pose2d startPose;

    private final DriverStation.Alliance alliance;

    public AutonomousBase(DriverStation.Alliance alliance, Pose2d blueStartPose) {
        this.alliance = alliance;
        startPose = createPose(blueStartPose);
    }

    @Override
    public List<Trajectory> getTrajectories() {
        return trajectories;
    }

    @Override
    public Pose2d getStartPose() {
        return startPose;
    }
    public static final double FIELD_LENGTH = Units.inchesToMeters(651.25);
    private static final Rotation2d RED_ROTATION = new Rotation2d(Math.PI);

    private final Function<Pose2d, Translation2d> RED_TRANSLATE = (Pose2d p) -> new Translation2d(FIELD_LENGTH - p.getX(), p.getY());
    private final Function<Translation2d, Translation2d> RED_ORIGIN = (Translation2d t) -> new Translation2d(FIELD_LENGTH - t.getX(), t.getY());
    private Pose2d  convertToRed(Pose2d pose) {
        return new Pose2d(
                RED_TRANSLATE.apply(pose),
                RED_ROTATION.minus(pose.getRotation()));
    }
    private List<Translation2d>  convertToRed(List<Translation2d> interiorWaypoints) {
        return interiorWaypoints.stream().map(w -> RED_ORIGIN.apply(w))
                .collect(Collectors.toList());
    }

    protected Trajectory createTrajectory(Pose2d start, Pose2d end, TrajectoryConfig config) {
        return createTrajectory(start, List.of(), end, config);
    }
    protected Trajectory createTrajectory(Pose2d start, List<Translation2d> interiorWaypoints, Pose2d end, TrajectoryConfig config) {
        Trajectory trajectory =  alliance == DriverStation.Alliance.Blue ?

                // blue has no conversion
                TrajectoryGenerator.generateTrajectory( start, interiorWaypoints, end, config) :

                // red needs to be converted
                TrajectoryGenerator.generateTrajectory(
                        convertToRed(start),
                        convertToRed(interiorWaypoints),
                        convertToRed(end),
                        config);

        trajectories.add(trajectory);

        return trajectory;
    }

    protected Rotation2d createRotation(Rotation2d angle) {
        if (alliance == DriverStation.Alliance.Red) {
            angle = RED_ROTATION.minus(angle);
        }
        return angle;
    }

    protected Pose2d createPose(Pose2d pose) {
        if (alliance != DriverStation.Alliance.Blue) {
            double x = Math.abs(FIELD_LENGTH - pose.getX());
            double radians = pose.getRotation().getRadians();
            Rotation2d theta = new Rotation2d(-Math.cos(radians), Math.sin(radians));
            pose = new Pose2d(x, pose.getY(), theta);
        }
        return pose;
    }
}
