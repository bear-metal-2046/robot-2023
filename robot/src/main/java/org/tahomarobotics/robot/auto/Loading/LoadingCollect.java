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
import org.tahomarobotics.robot.auto.TrajectoryCommand;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.grabber.IngestCommand;
import org.tahomarobotics.robot.grabber.ScoreCommand;

import java.util.List;

public class LoadingCollect extends AutonomousBase {
    public LoadingCollect(DriverStation.Alliance alliance) {
        final Pose2d FIRST_PLACE =
                new Pose2d(Units.inchesToMeters(69.6), Units.inchesToMeters(196.325), new Rotation2d(0));
        final Translation2d MID_PT =
                new Translation2d(Units.inchesToMeters(69.6 + 84.0), Units.inchesToMeters(196.325 - 8.0));
        final Pose2d FIRST_COLLECT =
                new Pose2d(Units.inchesToMeters(69.6 + 190.9), Units.inchesToMeters(196.325), new Rotation2d(0));
        final Rotation2d PLACE_HEADING = new Rotation2d(Units.degreesToRadians(-179));
        final Rotation2d COLLECT_HEADING = new Rotation2d(Units.degreesToRadians(0));

        final TrajectoryConfig CONFIG =
                new TrajectoryConfig(2.5, 3)
                        .setKinematics(Chassis.getInstance().getSwerveDriveKinematics());

        /////////////////////////
        initialize(alliance, new Pose2d(FIRST_PLACE.getTranslation(), PLACE_HEADING));

        // alliance converted trajectories
        Trajectory collectTrajectory = createTrajectory(FIRST_PLACE, List.of(MID_PT), FIRST_COLLECT, CONFIG);

        // alliance converted rotations
        Rotation2d collectHeading = createRotation(COLLECT_HEADING);

        TrajectoryCommand.TurnDirection turnDirection = alliance == DriverStation.Alliance.Blue ?
                TrajectoryCommand.TurnDirection.COUNTER_CLOCKWISE : TrajectoryCommand.TurnDirection.CLOCKWISE;

        addCommands(
                new InstantCommand(() -> Chassis.getInstance().resetOdometry(startPose)),
                ArmMovements.START_TO_HIGH_POLE.createArmWristMoveCommand(),
                new ScoreCommand(0.25),
                new ParallelCommandGroup(
                        new TrajectoryCommand("Start to collect", collectTrajectory, collectHeading, 0.3, 0.9, turnDirection),
                        new SequentialCommandGroup(
                                ArmMovements.HIGH_POLE_TO_STOW.createArmWristMoveCommand(),
                                new ParallelCommandGroup(
                                        ArmMovements.STOW_TO_CUBE_COLLECT.createArmWristMoveCommand(),
                                        new IngestCommand(1)
                                )
                        )
                ),
                new WaitCommand(0.25),
                ArmMovements.CUBE_COLLECT_TO_STOW.createArmWristMoveCommand()
        );
    }
}
