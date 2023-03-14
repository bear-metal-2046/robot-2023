package org.tahomarobotics.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.tahomarobotics.robot.arm.ArmMoveCommand;
import org.tahomarobotics.robot.arm.ArmMovements;
import org.tahomarobotics.robot.chassis.Chassis;

import java.util.ArrayList;
import java.util.List;

public class PlaceTaxi extends SequentialCommandGroup implements AutonomousCommandIF {

    private static final List<Trajectory> trajectories = new ArrayList<>();

    private final Pose2d startPose = new Pose2d(Units.inchesToMeters(582.9), Units.inchesToMeters(220.1),
            new Rotation2d(Units.degreesToRadians(180)));
    private final Pose2d firstPreCollect = new Pose2d(Units.inchesToMeters(434.2), Units.inchesToMeters(201.8),
            new Rotation2d(Units.degreesToRadians(180)));
    private final Rotation2d fistCollectRot = new Rotation2d(Units.degreesToRadians(180));
    public PlaceTaxi() {

        TrajectoryConfig config = new TrajectoryConfig(2, 3)
                .setKinematics(Chassis.getInstance().getSwerveDriveKinematics());

        addCommands(
                new InstantCommand(() -> Chassis.getInstance().resetOdometry(new Pose2d(startPose.getTranslation(), new Rotation2d(0)))),
                new ParallelCommandGroup(
                        Drive.drive("Reverse-to-Taxi", startPose, firstPreCollect, fistCollectRot, config, trajectories),
                        new ArmMoveCommand(ArmMovements.HIGH_POLE_TO_STOW)
                ),
                new InstantCommand(() -> Chassis.getInstance().resetOdometry(
                        new Pose2d(firstPreCollect.getTranslation(),
                        new Rotation2d(Units.degreesToRadians(180))))
                )
        );
    }


    @Override
    public Pose2d getStartPose() {
        Pose2d startPose = new Pose2d(this.startPose.getTranslation(), new Rotation2d());
        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
            startPose = AllianceUtil.convertToRed(startPose);
        }
        return startPose;
    }

    @Override
    public List<Trajectory> getTrajectories() {
        return List.of();
    }
}