package org.tahomarobotics.robot.auto.Cable;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import org.tahomarobotics.robot.arm.ArmMoveCommand;
import org.tahomarobotics.robot.arm.ArmMovements;
import org.tahomarobotics.robot.auto.*;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.grabber.IngestCommand;
import org.tahomarobotics.robot.grabber.ScoreCommand;

import java.util.List;

public class CableTwoPieceCollect extends AutonomousBase {
    // Poses
    private static final FudgeablePose START_POSE = FudgeablePose.newWithInches(69.6, 20.029, 0);

    private static final FudgeablePose FIRST_COLLECT = FudgeablePose.newWithInches(282.035, 36, 0)
            .withYFudgeInches(0, 0);

    private static final FudgeablePose CUBE_PLACE = FudgeablePose.newWithInches(69.6, 41.16, Math.PI)
            .withYFudgeInches(0, 0).withXFudgeInches(0, -20);

    private static final FudgeablePose SECOND_COLLECT = FudgeablePose.newWithInches(291.91, 96.81, Math.PI / 4)
            .withYFudgeInches(0, 0);


    // Midpoints
    private static final FudgeableTranslation BUMP_GOING_REV = FudgeableTranslation.newWithInches(133.10, 24.029);
    private static final FudgeableTranslation SECOND_COLLECT_MID = FudgeableTranslation.newWithInches(220, 40);

    // Headings
    private static final Rotation2d PLACE_HEADING = new Rotation2d(Math.PI);
    private static final Rotation2d COLLECT_HEADING = new Rotation2d(0);
    private static final Rotation2d SECOND_COLLECT_HEADING = new Rotation2d(Math.PI / 4);

    // Config(s)
    private static final TrajectoryConfig CONFIG = createConfig(3, 2.5);

    public CableTwoPieceCollect(DriverStation.Alliance alliance) {
        super(alliance, new Pose2d(START_POSE.getFudgedTranslation(alliance), PLACE_HEADING));

        Trajectory placeToCollect = createTrajectory(START_POSE, FIRST_COLLECT, CONFIG);
        Trajectory collectToPlace = createTrajectory(FIRST_COLLECT.getMirrored(), List.of(BUMP_GOING_REV), CUBE_PLACE, CONFIG);
        Trajectory placeToCollect2 = createTrajectory(CUBE_PLACE.getMirrored(), List.of(SECOND_COLLECT_MID), SECOND_COLLECT, CONFIG);

        Rotation2d collectHeading = createRotation(COLLECT_HEADING);
        Rotation2d collectHeading2 = createRotation(SECOND_COLLECT_HEADING);
        Rotation2d placeHeading = createRotation(PLACE_HEADING);

        TrajectoryCommand.TurnDirection turnDirection1 = TrajectoryCommand.TurnDirection.CLOCKWISE.reverseIfRed(alliance);
        TrajectoryCommand.TurnDirection turnDirection2 = TrajectoryCommand.TurnDirection.COUNTER_CLOCKWISE.reverseIfRed(alliance);

        Timer t = new Timer();

        addCommands(
                new InstantCommand(t::start),
                new InstantCommand(() -> Chassis.getInstance().resetOdometry(startPose)),
                ArmMovements.START_TO_HIGH_POLE.createArmWristMoveCommand(),
                new ScoreCommand(0.25),
                new ParallelCommandGroup(
                        new TrajectoryCommand("Start to Collect", placeToCollect, collectHeading, 0.0, 0.5,
                                turnDirection1),
                        new SequentialCommandGroup(
                                ArmMovements.HIGH_POLE_TO_STOW.createArmWristMoveCommand(),
                                new ParallelCommandGroup(
                                        ArmMovements.STOW_TO_CUBE_COLLECT.createArmWristMoveCommand(),
                                        new IngestCommand(2)
                                )
                        )
                ),
                new ParallelCommandGroup(
                        new TrajectoryCommand("Collect to Second Place", collectToPlace, placeHeading, 0.0, 0.6,
                                turnDirection2),
                        new SequentialCommandGroup(
                                ArmMovements.CUBE_COLLECT_TO_STOW.createArmWristMoveCommand(),
                                new WaitCommand(1),
                                ArmMovements.STOW_TO_HIGH_BOX.createArmWristMoveCommand()
                        )
                ),
                new ScoreCommand(0.25),
                new ParallelCommandGroup(
                        new TrajectoryCommand("Place to Second Collect", placeToCollect2, collectHeading2, 0.0, 0.5,
                                turnDirection1),
                        new SequentialCommandGroup(
                                ArmMovements.HIGH_POLE_TO_STOW.createArmWristMoveCommand(),
                                new ParallelCommandGroup(
                                        ArmMovements.STOW_TO_CUBE_COLLECT.createArmWristMoveCommand(),
                                        new IngestCommand(2)
                                )
                        )
                ),
                ArmMovements.CUBE_COLLECT_TO_STOW.createArmWristMoveCommand(),
                new InstantCommand(() -> {
                    t.stop();
                    DriverStation.reportError("Time taken: " + t.get(), false);
                })
        );
    }
}
