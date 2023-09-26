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
import org.tahomarobotics.robot.auto.AutonomousBase;
import org.tahomarobotics.robot.auto.FudgeablePose;
import org.tahomarobotics.robot.auto.FudgeableTranslation;
import org.tahomarobotics.robot.auto.TrajectoryCommand;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.grabber.IngestCommand;
import org.tahomarobotics.robot.grabber.ScoreCommand;

import java.util.List;

public class CableWeirdThreePiece extends AutonomousBase {
    public CableWeirdThreePiece(DriverStation.Alliance alliance) {
        // Poses
        final FudgeablePose START_POSE = FudgeablePose.newWithInches(73, 20.029, 0);

        final FudgeablePose FIRST_COLLECT = FudgeablePose.newWithInches(282.035, 38, 0)
                .withYFudgeInches(-11, -3).withXFudgeInches(6,0);

        final FudgeablePose CUBE_PLACE = FudgeablePose.newWithInches(73, 41.16, Math.PI)
                .withYFudgeInches(6, 6).withXFudgeInches(2, 0);

        final FudgeablePose PRE_BUMP = FudgeablePose.newWithInches(190, 26, Math.PI).withXFudgeInches(0,0);

        final FudgeablePose SECOND_COLLECT = FudgeablePose.newWithInches(291.91, 96.81, Math.PI / 4)
                .withYFudgeInches(2, 0);


        // Midpoints
        final FudgeableTranslation SECOND_COLLECT_MID = FudgeableTranslation.newWithInches(220, 40);

        // Headings
        final Rotation2d SHOOT_HEADING_1 = new Rotation2d(Math.PI);
        final Rotation2d SHOOT_HEADING_2 = new Rotation2d(Math.PI);
        final Rotation2d COLLECT_HEADING = new Rotation2d(0);
        final Rotation2d SECOND_COLLECT_HEADING = new Rotation2d(Math.PI / 6);

        // Config(s)
        final TrajectoryConfig CONFIG = createConfig(4, 2);
        final TrajectoryConfig FAST_CONFIG = createConfig(4, 3); // Was 5 4.5

        /////////////////////////
        initialize(alliance, new Pose2d(START_POSE.getFudgedTranslation(alliance), SHOOT_HEADING_1));

        Trajectory placeToCollect = createTrajectory(START_POSE, FIRST_COLLECT, FAST_CONFIG);
        Trajectory collectToShoot = createTrajectory(FIRST_COLLECT.getMirrored(), PRE_BUMP, FAST_CONFIG);
        Trajectory shootToCollect = createTrajectory(PRE_BUMP.getMirrored(), List.of(SECOND_COLLECT_MID), SECOND_COLLECT, FAST_CONFIG);
        Trajectory collectToPlace = createTrajectory(SECOND_COLLECT.getMirrored(), List.of(PRE_BUMP.getFudgedTranslation(alliance)),
                CUBE_PLACE, FAST_CONFIG);

        Rotation2d collectHeading1 = createRotation(COLLECT_HEADING);
        Rotation2d collectHeading2 = createRotation(SECOND_COLLECT_HEADING);
        Rotation2d shootHeading1 = createRotation(SHOOT_HEADING_1);
        Rotation2d shootHeading2 = createRotation(SHOOT_HEADING_2);

        TrajectoryCommand.TurnDirection turnDirection1 = TrajectoryCommand.TurnDirection.CLOCKWISE.reverseIfRed(alliance);
        TrajectoryCommand.TurnDirection turnDirection2 = TrajectoryCommand.TurnDirection.COUNTER_CLOCKWISE.reverseIfRed(alliance);

        Timer t = new Timer();

        addCommands(
                new InstantCommand(t::restart),
                new InstantCommand(() -> Chassis.getInstance().resetOdometry(startPose)),
                ArmMovements.START_TO_HIGH_POLE_FAST.createArmWristMoveCommand(),
                new WaitCommand(0.25),
                new ScoreCommand(0.25),
                new ParallelCommandGroup(
                        new TrajectoryCommand("Start to Collect", placeToCollect, collectHeading1, 0.0, 0.5,
                                turnDirection1, 0.0),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        ArmMovements.HIGH_POLE_TO_CUBE_COLLECT.createArmWristMoveCommand()
                                ),
                                new SequentialCommandGroup(
                                        new WaitCommand(1),
                                        new IngestCommand(2)
                                )
                        )
                ),
                new ParallelCommandGroup(
                        new TrajectoryCommand("Collect to Shoot 1", collectToShoot, shootHeading2, 0.0, 0.8,
                                turnDirection2, 0.0),
                        new SequentialCommandGroup(
                                new WaitCommand(1.5),
                                new ScoreCommand(0.5)
                        )
                ),
                new ParallelCommandGroup(
                        new TrajectoryCommand("Shoot to Second Collect", shootToCollect, collectHeading2, 0.0, 0.4,
                                turnDirection1, 0.0),
                        new SequentialCommandGroup(
                                new IngestCommand(2)
                        )
                ),
                new ParallelCommandGroup(
                        new TrajectoryCommand("Second Collect to Bump", collectToPlace, shootHeading1, 0.0, 0.6,
                                turnDirection2, 0.0),
                        new SequentialCommandGroup(
                                new WaitCommand(1.25),
                                new ParallelCommandGroup(
                                        ArmMovements.CUBE_COLLECT_TO_HIGH_POLE_FAST.createArmWristMoveCommand(),
                                        new SequentialCommandGroup(
                                                new WaitCommand(2),
                                                new ScoreCommand(0.25)
                                        )
                                )
                        )
                ),

                ArmMovements.HIGH_BOX_TO_STOW.createArmWristMoveCommand(),
                new InstantCommand(() -> {
                    t.stop();
                    logger.info("Time taken: " + t.get());
                })
        );
    }
}
