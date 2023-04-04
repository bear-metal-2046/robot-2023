package org.tahomarobotics.robot.auto.Loading;

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
import org.tahomarobotics.robot.auto.AutonomousBase;
import org.tahomarobotics.robot.auto.TrajectoryCommand;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.grabber.IngestCommand;
import org.tahomarobotics.robot.grabber.ScoreCommand;

import java.util.List;

public class LoadingTwoPiece extends AutonomousBase {

    private static final Pose2d FIRST_PLACE = new Pose2d(Units.inchesToMeters(69.6), Units.inchesToMeters(196.325),
            new Rotation2d(0));
    private static final Pose2d SECOND_PLACE = new Pose2d(Units.inchesToMeters(69.6), Units.inchesToMeters(196.325 - 24.0),
            new Rotation2d(Units.degreesToRadians(180)));
    private static final Translation2d MID_PT = new Translation2d(Units.inchesToMeters(69.6 + 84.0), Units.inchesToMeters(196.325 - 15.0));
    private static final Translation2d MID_PT_2 = new Translation2d(Units.inchesToMeters(69.6 + 80.0), Units.inchesToMeters(196.325 - 24));
    private static final Pose2d FIRST_COLLECT = new Pose2d(Units.inchesToMeters(69.6 + 200.9), Units.inchesToMeters(196.325 - 5),
            new Rotation2d(Units.degreesToRadians(-35)));
    private static final Pose2d FIRST_COLLECT_2 = new Pose2d(Units.inchesToMeters(69.6 + 200.9), Units.inchesToMeters(196.325 - 5),
            new Rotation2d(Units.degreesToRadians(180)));
    private static final Rotation2d PLACE_HEADING = new Rotation2d(Units.degreesToRadians(180));
    private static final Rotation2d COLLECT_HEADING = new Rotation2d(Units.degreesToRadians(-35));

    private static final TrajectoryConfig CONFIG = new TrajectoryConfig(2.0, 3)
            .setKinematics(Chassis.getInstance().getSwerveDriveKinematics());
    private static final TrajectoryConfig REVERSED_CONFIG = new TrajectoryConfig(2.0, 3)
            .setKinematics(Chassis.getInstance().getSwerveDriveKinematics()).setReversed(true);


    public LoadingTwoPiece(DriverStation.Alliance alliance) {

        // alliance converted start pose
        super(alliance, new Pose2d(FIRST_PLACE.getTranslation(), PLACE_HEADING));

        // alliance converted trajectories
        Trajectory collectTrajectory = createTrajectory(FIRST_PLACE, List.of(MID_PT), FIRST_COLLECT, CONFIG);
        Trajectory placeTrajectory = createTrajectory(FIRST_COLLECT_2, List.of(MID_PT_2), SECOND_PLACE, CONFIG);

        // alliance converted rotations
        Rotation2d collectHeading = createRotation(COLLECT_HEADING);
        Rotation2d placeHeading = createRotation(PLACE_HEADING);

        TrajectoryCommand.TurnDirection turnDirection1 = alliance == DriverStation.Alliance.Blue ?
                TrajectoryCommand.TurnDirection.COUNTER_CLOCKWISE : TrajectoryCommand.TurnDirection.CLOCKWISE;

        TrajectoryCommand.TurnDirection turnDirection2 = alliance == DriverStation.Alliance.Blue ?
                TrajectoryCommand.TurnDirection.CLOCKWISE : TrajectoryCommand.TurnDirection.COUNTER_CLOCKWISE;

        addCommands(
                new InstantCommand(() -> Chassis.getInstance().resetOdometry(startPose)),
                ArmMovements.START_TO_HIGH_POLE.createArmWristMoveCommand(),
                new ScoreCommand(0.25),
                new ParallelCommandGroup(
                        new TrajectoryCommand("Start to collect", collectTrajectory, collectHeading, 0.3, 0.8, turnDirection1),
                        new SequentialCommandGroup(
                                ArmMovements.HIGH_POLE_TO_STOW.createArmWristMoveCommand(),
                                new ParallelCommandGroup(
                                        ArmMovements.STOW_TO_CUBE_COLLECT.createArmWristMoveCommand(),
                                        new IngestCommand(2)
                                )
                        )
                ),
                new ParallelCommandGroup(
                        new TrajectoryCommand("Collect to Place", placeTrajectory, placeHeading, 0.2, 0.7, turnDirection2),
                        new SequentialCommandGroup(
                                ArmMovements.CUBE_COLLECT_TO_STOW.createArmWristMoveCommand(),
                                new WaitCommand(1),
                                ArmMovements.STOW_TO_HIGH_BOX.createArmWristMoveCommand()
                        )
                ),
                new ScoreCommand(0.25),
                ArmMovements.HIGH_BOX_TO_STOW.createArmWristMoveCommand()
        );
    }
}
