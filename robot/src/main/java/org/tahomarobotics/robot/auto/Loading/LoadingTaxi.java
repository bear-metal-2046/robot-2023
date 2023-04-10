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
import org.tahomarobotics.robot.arm.ArmMovements;
import org.tahomarobotics.robot.auto.AutonomousBase;
import org.tahomarobotics.robot.auto.TrajectoryCommand;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.grabber.ScoreCommand;

import java.util.List;

public class LoadingTaxi extends AutonomousBase {
    public LoadingTaxi(DriverStation.Alliance alliance) {
        final Pose2d FIRST_PLACE =
                new Pose2d(Units.inchesToMeters(69.6), Units.inchesToMeters(196.325), new Rotation2d(0));
        final Translation2d MID_PT =
                new Translation2d(Units.inchesToMeters(69.6 + 84.0), Units.inchesToMeters(196.325 - 8.0));
        final Pose2d TAXI =
                new Pose2d(Units.inchesToMeters(69.6 + 170), Units.inchesToMeters(196.325), new Rotation2d(0));
        final Rotation2d PLACE_HEADING = new Rotation2d(Units.degreesToRadians(-179));
        final Rotation2d TAXI_HEADING = new Rotation2d(Units.degreesToRadians(0));

        final TrajectoryConfig CONFIG = new TrajectoryConfig(2.5, 3)
                .setKinematics(Chassis.getInstance().getSwerveDriveKinematics());

        ///////////////////////////
        initialize(alliance, new Pose2d(FIRST_PLACE.getTranslation(), PLACE_HEADING));

        // alliance converted trajectories
        Trajectory taxiTrajectory = createTrajectory(FIRST_PLACE, List.of(MID_PT), TAXI, CONFIG);

        // alliance converted rotations
        Rotation2d taxiHeading = createRotation(TAXI_HEADING);

        TrajectoryCommand.TurnDirection turnDirection = alliance == DriverStation.Alliance.Blue ?
                TrajectoryCommand.TurnDirection.COUNTER_CLOCKWISE : TrajectoryCommand.TurnDirection.CLOCKWISE;

        addCommands(
                new InstantCommand(() -> Chassis.getInstance().resetOdometry(startPose)),
                ArmMovements.STOW_TO_HIGH_POLE.createArmWristMoveCommand(),
                new ScoreCommand(0.25),

                new ParallelCommandGroup(
                        new TrajectoryCommand("Reverse-to-Taxi", taxiTrajectory, taxiHeading, 0.3, 0.9, turnDirection),
                        ArmMovements.HIGH_POLE_TO_STOW.createArmWristMoveCommand()
                )
        );
    }
}