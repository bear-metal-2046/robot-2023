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
    // Poses
    private static final FudgeablePose START_POSE = FudgeablePose.newWithInches(69.6, 20.029, 0);

    private static final FudgeablePose FIRST_COLLECT = FudgeablePose.newWithInches(282.035, 38, 0)
            .withYFudgeInches(-5, -3).withXFudgeInches(6,0);

    private static final FudgeablePose CUBE_PLACE = FudgeablePose.newWithInches(69.6, 41.16, Math.PI)
            .withYFudgeInches(6, 6).withXFudgeInches(0, -20);

    private static final FudgeablePose PRE_BUMP = FudgeablePose.newWithInches(190, 26, Math.PI).withXFudgeInches(8,0);

    private static final FudgeablePose SECOND_COLLECT = FudgeablePose.newWithInches(291.91, 96.81, Math.PI / 4)
            .withYFudgeInches(-12, 0);


    // Midpoints
    private static final FudgeableTranslation SECOND_COLLECT_MID = FudgeableTranslation.newWithInches(220, 40);

    // Headings
    private static final Rotation2d SHOOT_HEADING_1 = new Rotation2d(Math.PI);
    private static final Rotation2d SHOOT_HEADING_2 = new Rotation2d(Math.PI * 0.95);
    private static final Rotation2d COLLECT_HEADING = new Rotation2d(0);
    private static final Rotation2d SECOND_COLLECT_HEADING = new Rotation2d(Math.PI / 6);

    // Config(s)
    private static final TrajectoryConfig CONFIG = createConfig(4, 2);
    private static final TrajectoryConfig NOT_AS_FAST_CONFIG = createConfig(4, 3);
    private static final TrajectoryConfig FAST_CONFIG = createConfig(5, 4.5);

    public CableWeirdThreePiece(DriverStation.Alliance alliance) {
        super(alliance, new Pose2d(START_POSE.getFudgedTranslation(alliance), SHOOT_HEADING_1));

        Trajectory placeToCollect = createTrajectory(START_POSE, FIRST_COLLECT, CONFIG);
        Trajectory collectToShoot = createTrajectory(FIRST_COLLECT.getMirrored(), PRE_BUMP, FAST_CONFIG);
        Trajectory shootToCollect = createTrajectory(PRE_BUMP.getMirrored(), List.of(SECOND_COLLECT_MID), SECOND_COLLECT, FAST_CONFIG);
        Trajectory collectToPlace = createTrajectory(SECOND_COLLECT.getMirrored(), List.of(PRE_BUMP.getFudgedTranslation(alliance)),
                CUBE_PLACE, CONFIG);

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
                                new WaitCommand(1),
                                new ParallelCommandGroup(
                                        ArmMovements.CUBE_COLLECT_TO_HIGH_POLE_FAST.createArmWristMoveCommand(),
                                        new SequentialCommandGroup(
                                                new WaitCommand(2.25),
                                                new ScoreCommand(0.25)
                                        )
                                )
                        )
                ),

                ArmMovements.HIGH_BOX_TO_STOW.createArmWristMoveCommand(),
                new InstantCommand(() -> {
                    t.stop();
                    DriverStation.reportError("Time taken: " + t.get(), false);
                })
        );
    }
}
