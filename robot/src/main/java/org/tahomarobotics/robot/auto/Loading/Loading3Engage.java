package org.tahomarobotics.robot.auto.Loading;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.tahomarobotics.robot.arm.ArmMovements;
import org.tahomarobotics.robot.auto.AutonomousBase;
import org.tahomarobotics.robot.auto.FudgeablePose;
import org.tahomarobotics.robot.auto.FudgeableTranslation;
import org.tahomarobotics.robot.auto.TrajectoryCommand;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.grabber.IngestCommand;
import org.tahomarobotics.robot.grabber.ScoreCommand;

import java.util.List;

public class Loading3Engage extends AutonomousBase {

    public Loading3Engage(DriverStation.Alliance alliance) {
        //Place Points
        final FudgeablePose FIRST_PLACE = FudgeablePose.newWithInchesAndDegreesForZach(69.6, 196.325, 0);

        final FudgeablePose SECOND_PLACE = FudgeablePose.newWithInchesAndDegreesForZach(71.6, 196.325 - 24.0, 180)
                .withYFudgeInches(0, 0);
        final FudgeablePose SECOND_PLACE_MIRRORED = SECOND_PLACE.getMirrored();
        final FudgeablePose ENGAGE = FudgeablePose.newWithInchesAndDegreesForZach(69.6 + 40.0, 125.0, 180);

        //Mid-Translations
        final FudgeableTranslation FIRST_GOING_OUT = FudgeableTranslation.newWithInches(153.6, 186.325);
        final FudgeableTranslation FIRST_GOING_AROUND = FudgeableTranslation.newWithInches(267, 220.6);
        final FudgeableTranslation FIRST_GOING_AROUND_2 = FudgeableTranslation.newWithInches(297, 206);
        final FudgeableTranslation FIRST_GOING_COLLECT = FudgeableTranslation.newWithInches(267, 180);

        final FudgeableTranslation SECOND_GOING_AROUND = FudgeableTranslation.newWithInches(276, 177);
        final FudgeableTranslation SECOND_GOING_COLLECT = FudgeableTranslation.newWithInches(271, 130);

        final Rotation2d PLACE_HEADING = new Rotation2d(Units.degreesToRadians(180));


        final TrajectoryConfig CONFIG_GOING = createConfig(3., 3);
        final TrajectoryConfig PLZ_MAKE_IT_BACK_CONFIG = new TrajectoryConfig(3.5, 3)
                .setKinematics(Chassis.getInstance().getSwerveDriveKinematics());

        /////////////////////////
        initialize(alliance, new Pose2d(FIRST_PLACE.getTranslation(), PLACE_HEADING));

        // alliance converted trajectories
        Trajectory collectTrajectory1 = createTrajectory(FIRST_PLACE, List.of(FIRST_GOING_OUT, FIRST_GOING_AROUND, FIRST_GOING_AROUND_2, FIRST_GOING_COLLECT, FIRST_GOING_OUT), SECOND_PLACE, CONFIG_GOING);
        Trajectory collectAndBalance = createTrajectory(SECOND_PLACE_MIRRORED, List.of(FIRST_GOING_OUT, SECOND_GOING_AROUND, SECOND_GOING_COLLECT), ENGAGE, CONFIG_GOING);

        Timer t = new Timer();

        addCommands(
                new InstantCommand(t::restart),
                new InstantCommand(() -> Chassis.getInstance().resetOdometry(startPose)),
                ArmMovements.START_TO_HIGH_POLE.createArmWristMoveCommand(),
                new ScoreCommand(0.25),
                new ParallelCommandGroup(
                        new TrajectoryCommand("First Collect",
                                collectTrajectory1,
                                createRotationPath(
                                        new TrajectoryCommand.MidRotation(.35d, Rotation2d.fromDegrees(180d)),
                                        new TrajectoryCommand.MidRotation(.55d, Rotation2d.fromDegrees(225d)),
                                        new TrajectoryCommand.MidRotation(.75d, Rotation2d.fromDegrees(180d))
                                ),
                                0.0),
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        ArmMovements.HIGH_POLE_TO_CUBE_COLLECT.createArmWristMoveCommand(),
                                        new SequentialCommandGroup(
                                                new WaitCommand(2),
                                                new IngestCommand(2)
                                        )
                                ),
                                ArmMovements.CUBE_COLLECT_TO_HIGH_POLE_FAST.createArmWristMoveCommand(),
                                new ScoreCommand(0.1)
                        )
                ),
                new ParallelCommandGroup(
                        new TrajectoryCommand("Second Collect",
                                collectAndBalance,
                                createRotationPath(
                                        new TrajectoryCommand.MidRotation(.25d, Rotation2d.fromDegrees(180d)),
                                        new TrajectoryCommand.MidRotation(.60d, Rotation2d.fromDegrees(235d)),
                                        new TrajectoryCommand.MidRotation(.80d, Rotation2d.fromDegrees(180d))
                                ),
                                0.0),
                        new SequentialCommandGroup(
                                ArmMovements.HIGH_BOX_TO_CUBE_COLLECT.createArmWristMoveCommand(),
                                new IngestCommand(1.5),
                                ArmMovements.CUBE_COLLECT_TO_STOW.createArmWristMoveCommand(),
                                new InstantCommand(() -> {
                                    DriverStation.reportError("Time taken: " + t.get(), false);
                                }),
                                new ParallelCommandGroup(
                                    ArmMovements.STOW_TO_HIGH_POLE.createArmWristMoveCommand(),
                                    new SequentialCommandGroup(
                                            new WaitCommand(0.5),
                                            new ScoreCommand(0.25)
                                    )
                                )
                        )
                ),
                new InstantCommand(() -> {
                    t.stop();
                    DriverStation.reportError("Time taken: " + t.get(), false);
                })
        );
    }
}
