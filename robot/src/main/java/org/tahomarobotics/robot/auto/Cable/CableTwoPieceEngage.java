package org.tahomarobotics.robot.auto.Cable;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
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

public class CableTwoPieceEngage extends AutonomousBase {
    public CableTwoPieceEngage(DriverStation.Alliance alliance) {
        // Points
        final FudgeablePose START_POSE = FudgeablePose.newWithInches(69.6, 20.029, 0);

        final FudgeablePose FIRST_COLLECT = FudgeablePose.newWithInches(282.035, 38, 0)
                .withYFudgeInches(-5, -3).withXFudgeInches(6,0);

        final FudgeablePose CUBE_PLACE = FudgeablePose.newWithInches(69.6, 41.16, Math.PI)
                .withYFudgeInches(6, 6).withXFudgeInches(0, -20);

        final FudgeablePose SECOND_COLLECT = FudgeablePose.newWithInches(291.91, 96.81, Math.PI / 4)
                .withYFudgeInches(-12, 0);

        final FudgeableTranslation ENGAGE_MID = FudgeableTranslation.newWithInches(90, 80);

        final FudgeablePose ENGAGE = FudgeablePose.newWithInches(166.75, 89, 0);


        // Headings
        final Rotation2d SHOOT_HEADING_1 = new Rotation2d(Math.PI);
        final Rotation2d COLLECT_HEADING = new Rotation2d(0);

        // Config(s)
        final TrajectoryConfig CONFIG = createConfig(4, 2);
        final TrajectoryConfig FAST_CONFIG = createConfig(5, 4.5);

        /////////////////////////
        initialize(alliance, new Pose2d(START_POSE.getFudgedTranslation(alliance), SHOOT_HEADING_1));

        Trajectory placeToCollect = createTrajectory(START_POSE, FIRST_COLLECT, CONFIG);
        Trajectory collectToPlace = createTrajectory(FIRST_COLLECT.getMirrored(), CUBE_PLACE, CONFIG);
        Trajectory placeToEngage = createTrajectory(CUBE_PLACE.getMirrored(), List.of(ENGAGE_MID), ENGAGE, FAST_CONFIG);

        Rotation2d collectHeading1 = createRotation(COLLECT_HEADING);
        Rotation2d shootHeading1 = createRotation(SHOOT_HEADING_1);

        TrajectoryCommand.TurnDirection turnDirection1 = TrajectoryCommand.TurnDirection.CLOCKWISE.reverseIfRed(alliance);
        TrajectoryCommand.TurnDirection turnDirection2 = TrajectoryCommand.TurnDirection.COUNTER_CLOCKWISE.reverseIfRed(alliance);

        Timer t = new Timer();

        addCommands(
                new InstantCommand(t::restart),
                new InstantCommand(() -> Chassis.getInstance().resetOdometry(startPose)),
                ArmMovements.START_TO_HIGH_POLE_FAST.createArmWristMoveCommand(),
                new ScoreCommand(0.25),
                new ParallelCommandGroup(
                        new TrajectoryCommand("Start to Collect", placeToCollect, collectHeading1, 0.0, 0.5,
                                turnDirection1, 0.0),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new WaitCommand(0.5),
                                        ArmMovements.HIGH_POLE_TO_CUBE_COLLECT.createArmWristMoveCommand()
                                ),
                                new SequentialCommandGroup(
                                        new WaitCommand(1),
                                        new IngestCommand(2)
                                )
                        )
                ),
                new ParallelCommandGroup(
                        new TrajectoryCommand("Collect to Place", collectToPlace, shootHeading1, 0.0, 0.8,
                                turnDirection2, 0.0),
                        new SequentialCommandGroup(
                                ArmMovements.CUBE_COLLECT_TO_STOW.createArmWristMoveCommand(),
                                new WaitCommand(0.5),
                                ArmMovements.STOW_TO_HIGH_BOX.createArmWristMoveCommand(),
                                new ScoreCommand(0.5)
                        )
                ),
                new ParallelCommandGroup(
                        ArmMovements.HIGH_BOX_TO_STOW.createArmWristMoveCommand(),
                         new TrajectoryCommand("Place to Engage", placeToEngage, shootHeading1, 0.0, 0.0,
                                turnDirection1, 0.0)
                ),
                new BalancedCommand(),
                new InstantCommand(() -> {
                    t.stop();
                    logger.info("Time taken: " + t.get());
                })
        );
    }
}
