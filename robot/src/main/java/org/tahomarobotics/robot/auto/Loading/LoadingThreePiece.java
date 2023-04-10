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
import org.tahomarobotics.robot.arm.ArmMovements;
import org.tahomarobotics.robot.auto.AutonomousBase;
import org.tahomarobotics.robot.auto.FudgeablePose;
import org.tahomarobotics.robot.auto.TrajectoryCommand;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.grabber.IngestCommand;
import org.tahomarobotics.robot.grabber.ScoreCommand;

import java.util.List;

public class LoadingThreePiece extends AutonomousBase {
    public LoadingThreePiece(DriverStation.Alliance alliance) {
        //Place Points
        final FudgeablePose FIRST_PLACE = FudgeablePose.newWithInchesAndDegreesForZach(69.6, 196.325, 0);
        final FudgeablePose SECOND_PLACE = FudgeablePose.newWithInchesAndDegreesForZach(71.6, 196.325 - 28.0, 180)
                .withYFudgeInches(-4, 0);
        final FudgeablePose SECOND_PLACE_PT_2 = SECOND_PLACE.getMirrored();
        final FudgeablePose SECOND_PLACE_PT_3 = FudgeablePose.newWithInchesAndDegreesForZach(71.6, 196.325 - 34.0, 180);


        final FudgeablePose FIRST_COLLECT = FudgeablePose.newWithInchesAndDegreesForZach(69.6 + 210.9, 196.325 - 11, -35)
                .withYFudgeInches(-2, -3);
        final FudgeablePose FIRST_COLLECT_PT_2 = FudgeablePose.newWithInchesAndDegreesForZach(69.6 + 210.9, 196.325 - 11, 180)
                .withYFudgeInches(-2, -3);
        final FudgeablePose SECOND_COLLECT = FudgeablePose.newWithInchesAndDegreesForZach(69.6 + 210.9, 196.325 - 67, 0)
                .withYFudgeInches(-4, 5);
        final FudgeablePose SECOND_COLLECT_PT_2 = FudgeablePose.newWithInchesAndDegreesForZach(69.6 + 210.9, 196.325 - 67, 145)
                .withYFudgeInches(-4, 5);


        //Mid-Translations
        final Translation2d MID_PT = new Translation2d(Units.inchesToMeters(69.6 + 84.0), Units.inchesToMeters(196.325 - 10.0));
        final Translation2d MID_PT_3 = new Translation2d(Units.inchesToMeters(109), Units.inchesToMeters(177));
        final Translation2d MID_PT_4 = new Translation2d(Units.inchesToMeters(206), Units.inchesToMeters(177));

        final Rotation2d PLACE_HEADING = new Rotation2d(Units.degreesToRadians(180));
        final Rotation2d SECOND_PLACE_HEADING = new Rotation2d(Units.degreesToRadians(180));
        final Rotation2d COLLECT_HEADING = new Rotation2d(Units.degreesToRadians(-35));
        final Rotation2d SECOND_COLLECT_HEADING = new Rotation2d(Units.degreesToRadians(-10));

        final TrajectoryConfig CONFIG = new TrajectoryConfig(2.8, 3)
                .setKinematics(Chassis.getInstance().getSwerveDriveKinematics());
        final TrajectoryConfig PLZ_MAKE_IT_BACK_CONFIG = new TrajectoryConfig(3.5, 3)
                .setKinematics(Chassis.getInstance().getSwerveDriveKinematics());

        /////////////////////////
        initialize(alliance, new Pose2d(FIRST_PLACE.getTranslation(), PLACE_HEADING));

        // alliance converted trajectories
        Trajectory collectTrajectory = createTrajectory(FIRST_PLACE, List.of(MID_PT), FIRST_COLLECT, CONFIG);
        Trajectory placeTrajectory = createTrajectory(FIRST_COLLECT_PT_2, List.of(MID_PT_4, MID_PT_3), SECOND_PLACE, CONFIG);
        Trajectory secondCollectTrajectory = createTrajectory(SECOND_PLACE_PT_2, List.of(MID_PT_3, MID_PT_4), SECOND_COLLECT, CONFIG);
        Trajectory secondPlaceTrajectory = createTrajectory(SECOND_COLLECT_PT_2, List.of(MID_PT_4, MID_PT_3), SECOND_PLACE_PT_3, PLZ_MAKE_IT_BACK_CONFIG);

        // alliance converted rotations
        Rotation2d collectHeading = createRotation(COLLECT_HEADING);
        Rotation2d secondCollectHeading = createRotation(SECOND_COLLECT_HEADING);
        Rotation2d placeHeading = createRotation(PLACE_HEADING);
        Rotation2d secondPlaceHeading = createRotation(SECOND_PLACE_HEADING);

        addCommands(
                new InstantCommand(() -> Chassis.getInstance().resetOdometry(startPose)),
                ArmMovements.START_TO_HIGH_POLE.createArmWristMoveCommand(),
                new ScoreCommand(0.25),
                new ParallelCommandGroup(
                        new TrajectoryCommand("Start to collect",
                                collectTrajectory,
                                collectHeading,
                                0.0, 0.5,
                                alliance == DriverStation.Alliance.Blue ? TrajectoryCommand.TurnDirection.COUNTER_CLOCKWISE : TrajectoryCommand.TurnDirection.CLOCKWISE,
                                0.0),
                        new SequentialCommandGroup(
                                ArmMovements.HIGH_POLE_TO_CUBE_COLLECT.createArmWristMoveCommand(),
                                new IngestCommand(1.2)
                        )
                ),
                new ParallelCommandGroup(
                        new TrajectoryCommand("Collect to Place",
                                placeTrajectory,
                                placeHeading,
                                0.0, 0.4,
                                alliance == DriverStation.Alliance.Blue ? TrajectoryCommand.TurnDirection.CLOCKWISE : TrajectoryCommand.TurnDirection.COUNTER_CLOCKWISE,
                                0.0),
                        new SequentialCommandGroup(
                                ArmMovements.CUBE_COLLECT_TO_STOW.createArmWristMoveCommand(),
                                new WaitCommand(0.125),
                                ArmMovements.STOW_TO_HIGH_BOX.createArmWristMoveCommand(),
                                new ScoreCommand(0.1)
                        )
                ),
                new ParallelCommandGroup(
                        new TrajectoryCommand("Place to 2nd Collect",
                                secondCollectTrajectory,
                                secondCollectHeading,
                                0.0, 0.5,
                                alliance == DriverStation.Alliance.Blue ? TrajectoryCommand.TurnDirection.COUNTER_CLOCKWISE : TrajectoryCommand.TurnDirection.CLOCKWISE,
                                0.0),
                        new SequentialCommandGroup(
                                ArmMovements.HIGH_BOX_TO_CUBE_COLLECT.createArmWristMoveCommand(),
                                new ParallelCommandGroup(
                                        new IngestCommand(1.2)
                                )
                        )
                ),
                new ParallelCommandGroup(
                        new TrajectoryCommand("Second Collect to Second Place",
                                secondPlaceTrajectory,
                                secondPlaceHeading,
                                0.0, 0.3,
                                alliance == DriverStation.Alliance.Blue ? TrajectoryCommand.TurnDirection.CLOCKWISE : TrajectoryCommand.TurnDirection.COUNTER_CLOCKWISE,
                                0.0),
                        new SequentialCommandGroup(
                                ArmMovements.CUBE_COLLECT_TO_STOW.createArmWristMoveCommand(),
                                new ParallelCommandGroup(
                                        ArmMovements.STOW_TO_MID_BOX.createArmWristMoveCommand(),
                                        new SequentialCommandGroup(
                                                new WaitCommand(1.2),
                                                new ScoreCommand(0.1)
                                        )
                                )
                        )
                ),
                ArmMovements.MID_BOX_TO_STOW.createArmWristMoveCommand()
        );
    }
}
