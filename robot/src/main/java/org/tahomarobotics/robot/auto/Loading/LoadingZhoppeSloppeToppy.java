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
import org.tahomarobotics.robot.auto.*;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.grabber.IngestCommand;
import org.tahomarobotics.robot.grabber.ScoreCommand;

import java.util.List;

public class LoadingZhoppeSloppeToppy extends AutonomousBase {
    public LoadingZhoppeSloppeToppy(DriverStation.Alliance alliance) {

        final FudgeablePose FIRST_PLACE = FudgeablePose.newWithInchesAndDegreesForZach(73, 196.325, 0);

        final FudgeablePose SECOND_PLACE = FudgeablePose.newWithInchesAndDegreesForZach(73, 196.325 - 24.0, 180)
                .withYFudgeInches(0, 0);
        final FudgeablePose ENGAGE = FudgeablePose.newWithInchesAndDegreesForZach(69.6 + 93.0, 125, 0);

        //Mid-Translations
        final FudgeableTranslation FIRST_GOING_OUT = FudgeableTranslation.newWithInches(153.6, 186.325);
        final FudgeableTranslation FIRST_GOING_AROUND = FudgeableTranslation.newWithInches(260, 218.6);
        final FudgeableTranslation FIRST_GOING_AROUND_2 = FudgeableTranslation.newWithInches(290, 206);
        final FudgeableTranslation FIRST_GOING_COLLECT = FudgeableTranslation.newWithInches(260, 178);

        final FudgeableTranslation ENGAGE_MID = FudgeableTranslation.newWithInches(69.6 + 12.0, 125.0);

        final Rotation2d PLACE_HEADING = new Rotation2d(Units.degreesToRadians(180));

        final TrajectoryConfig CONFIG_GOING = createConfig(3.5, 3);
        final TrajectoryConfig PLZ_MAKE_IT_BACK_CONFIG = new TrajectoryConfig(3.5, 3)
                .setKinematics(Chassis.getInstance().getSwerveDriveKinematics());

        /////////////////////////
        initialize(alliance, new Pose2d(FIRST_PLACE.getTranslation(), PLACE_HEADING));

        // alliance converted trajectories
        Trajectory collectTrajectory1 = createTrajectory(FIRST_PLACE, List.of(FIRST_GOING_OUT, FIRST_GOING_AROUND, FIRST_GOING_AROUND_2, FIRST_GOING_COLLECT, FIRST_GOING_OUT), SECOND_PLACE, CONFIG_GOING);
        Trajectory engageTrajectory1 = createTrajectory(SECOND_PLACE.getMirrored(), List.of(ENGAGE_MID), ENGAGE, CONFIG_GOING);

        Rotation2d placeHeading = createRotation(PLACE_HEADING);

        Timer t = new Timer();

        addCommands(
                new InstantCommand(t::restart),
                new InstantCommand(() -> Chassis.getInstance().resetOdometry(startPose)),
                ArmMovements.START_TO_HIGH_POLE.createArmWristMoveCommand(),
                new WaitCommand(0.30),
                new ScoreCommand(0.25),
                new ParallelCommandGroup(
                        new TrajectoryCommand("First Collect",
                                collectTrajectory1,
                                createRotationPath(
                                        new TrajectoryCommand.MidRotation(.35d, Rotation2d.fromDegrees(180d)),
                                        new TrajectoryCommand.MidRotation(.50d, Rotation2d.fromDegrees(235d)),
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
                        ArmMovements.HIGH_BOX_TO_STOW.createArmWristMoveCommand(),
                        new TrajectoryCommand("Place to Engage", engageTrajectory1, placeHeading)

                ),
                new BalancedCommand()
        );
    }
}
