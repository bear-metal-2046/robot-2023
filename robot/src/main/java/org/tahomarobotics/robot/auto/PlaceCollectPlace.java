package org.tahomarobotics.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.tahomarobotics.robot.arm.ArmMoveCommand;
import org.tahomarobotics.robot.arm.ArmMovements;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.grabber.IngestCommand;
import org.tahomarobotics.robot.grabber.ScoreCommand;

import java.util.ArrayList;
import java.util.List;

public class PlaceCollectPlace extends SequentialCommandGroup implements AutonomousCommandIF {
    private List<Trajectory> trajectories = new ArrayList<>();

    private static final Pose2d FIRST_PLACE = new Pose2d(Units.inchesToMeters(69.6), Units.inchesToMeters(196.325),
            new Rotation2d(0));
    private static final Pose2d SECOND_PLACE = new Pose2d(Units.inchesToMeters(69.6), Units.inchesToMeters(196.325 - 6),
            new Rotation2d(0));
    private static final Translation2d MID_PT = new Translation2d(Units.inchesToMeters(69.6 + 84.0), Units.inchesToMeters(196.325 - 8.0));
    private static final Translation2d MID_PT_2 = new Translation2d(Units.inchesToMeters(69.6 + 50.0), Units.inchesToMeters(196.325 - 8.0));
    private static final Pose2d FIRST_COLLECT = new Pose2d(Units.inchesToMeters(69.6 + 190.9), Units.inchesToMeters(196.325),
            new Rotation2d(0));
    private static final Rotation2d PLACE_HEADING = new Rotation2d(Units.degreesToRadians(-179));
    private static final Rotation2d COLLECT_HEADING = new Rotation2d(Units.degreesToRadians(0));

    private static final TrajectoryConfig CONFIG = new TrajectoryConfig(2.0, 3)
            .setKinematics(Chassis.getInstance().getSwerveDriveKinematics());
    private static final TrajectoryConfig REVERSED_CONFIG = new TrajectoryConfig(2.0, 3)
            .setKinematics(Chassis.getInstance().getSwerveDriveKinematics()).setReversed(true);


    private final Pose2d startPose;

    public PlaceCollectPlace(DriverStation.Alliance alliance) {

        startPose = AllianceUtil.adjustPoseForAlliance(alliance, new Pose2d(FIRST_PLACE.getTranslation(), PLACE_HEADING));

        Trajectory collectTrajectory = AllianceUtil.createTrajectory(alliance, FIRST_PLACE, List.of(MID_PT), FIRST_COLLECT, CONFIG);

        Trajectory placeTrajectory = AllianceUtil.createTrajectory(alliance, FIRST_COLLECT, List.of(MID_PT_2), SECOND_PLACE, REVERSED_CONFIG);

        Rotation2d collectHeading = AllianceUtil.adjustAngleForAlliance(alliance, COLLECT_HEADING);

        Rotation2d placeHeading = AllianceUtil.adjustAngleForAlliance(alliance, PLACE_HEADING);

        addCommands(
                new InstantCommand(() -> Chassis.getInstance().resetOdometry(startPose)),
                new ArmMoveCommand(ArmMovements.STOW_TO_HIGH_POLE),
                new ScoreCommand(0.25),
                new ParallelCommandGroup(
                        Drive.drive("Start to collect", collectTrajectory, collectHeading, trajectories),
                        new SequentialCommandGroup(
                                new ArmMoveCommand(ArmMovements.HIGH_POLE_TO_STOW),
                                new ParallelCommandGroup(
                                        new ArmMoveCommand(ArmMovements.STOW_TO_CUBE_COLLECT),
                                        new IngestCommand(1)
                                )
                        )
                ),
                new WaitCommand(0.25),
                new ParallelCommandGroup(
                        Drive.drive("Collect to Place", placeTrajectory, placeHeading, trajectories),
                        new SequentialCommandGroup(
                                new ArmMoveCommand(ArmMovements.CUBE_COLLECT_TO_STOW),
                                new WaitCommand(1),
                                new ArmMoveCommand(ArmMovements.STOW_TO_HIGH_BOX)
                        )
                ),
                new ScoreCommand(0.25),
                new ArmMoveCommand(ArmMovements.HIGH_BOX_TO_STOW)
        );
    }

    @Override
    public Pose2d getStartPose() {
        return startPose;
    }

    @Override
    public List<Trajectory> getTrajectories() {
        return trajectories;
    }
}
