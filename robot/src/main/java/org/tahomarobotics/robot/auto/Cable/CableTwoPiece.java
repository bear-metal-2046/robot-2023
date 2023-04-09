package org.tahomarobotics.robot.auto.Cable;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.tahomarobotics.robot.arm.ArmMoveCommand;
import org.tahomarobotics.robot.arm.ArmMovements;
import org.tahomarobotics.robot.auto.*;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.grabber.IngestCommand;
import org.tahomarobotics.robot.grabber.ScoreCommand;

import java.util.List;

public class CableTwoPiece extends AutonomousBase {
    private static final FudgeablePose START_POSE = FudgeablePose.newWithInches(69.6, 20.029, 0);

    private static final FudgeablePose FIRST_COLLECT = FudgeablePose.newWithInches(282.035, 36, 0)
            .withYFudgeInches(-7, 0);

    private static final FudgeablePose SECOND_PLACE = FudgeablePose.newWithInches(63.6, 38.029, Math.PI)
            .withYFudgeInches(0, 10);

    private static final FudgeablePose BUMP_GOING_REV = FudgeablePose.newWithInches(133.10, 24.029, Math.PI);

    private static final Rotation2d PLACE_HEADING = new Rotation2d(Math.PI);
    private static final Rotation2d COLLECT_HEADING = new Rotation2d(0);

    private static final TrajectoryConfig CONFIG = createConfig(3, 1.5);

    public CableTwoPiece(DriverStation.Alliance alliance) {
        super(alliance, new Pose2d(START_POSE.getFudgedTranslation(alliance), PLACE_HEADING));

        Trajectory placeToCollect = createTrajectory(START_POSE, FIRST_COLLECT, CONFIG);
        Trajectory collectToPlace = createTrajectory(FIRST_COLLECT.getMirrored(), List.of(BUMP_GOING_REV.getTranslation()), SECOND_PLACE, CONFIG);

        Rotation2d collectHeading = createRotation(COLLECT_HEADING);
        Rotation2d placeHeading = createRotation(PLACE_HEADING);

        TrajectoryCommand.TurnDirection turnDirection1 = TrajectoryCommand.TurnDirection.CLOCKWISE.reverseIfRed(alliance);
        TrajectoryCommand.TurnDirection turnDirection2 = TrajectoryCommand.TurnDirection.COUNTER_CLOCKWISE.reverseIfRed(alliance);

        addCommands(
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
                ArmMovements.HIGH_BOX_TO_STOW.createArmWristMoveCommand()
        );
    }
}
