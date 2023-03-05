package org.tahomarobotics.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.tahomarobotics.robot.chassis.Chassis;

import java.util.ArrayList;
import java.util.List;

public class PlaceTaxi extends Place implements AutonomousCommandIF {
    private List<Trajectory> trajectories = new ArrayList<>();
    private final Pose2d startPose = new Pose2d(Units.inchesToMeters(582.9), Units.inchesToMeters(216.7),
            new Rotation2d(Units.degreesToRadians(180)));
    private final Pose2d taxi = new Pose2d(Units.inchesToMeters(498.36), Units.inchesToMeters(216.7),
            new Rotation2d(Units.degreesToRadians(180)));
    private final Pose2d firstCollect = new Pose2d(Units.inchesToMeters(399.2), Units.inchesToMeters(201.8),
            new Rotation2d(Units.degreesToRadians(180)));
    private final Rotation2d taxiRot = new Rotation2d(Units.degreesToRadians(0));
    private final Rotation2d fistCollectRot = new Rotation2d(Units.degreesToRadians(180));
    public PlaceTaxi(GamePiece piece, Level level) {
        super(piece, level);
        TrajectoryConfig config = new TrajectoryConfig(1.5, 3)
                .setKinematics(Chassis.getInstance().getSwerveDriveKinematics());
        addCommands(
                new InstantCommand(() -> Chassis.getInstance().resetOdometry(new Pose2d(startPose.getTranslation(), new Rotation2d(0)))),
                Drive.drive(startPose, taxi, taxiRot, config.setEndVelocity(1.5), trajectories),
                Drive.drive(taxi, firstCollect, fistCollectRot, config.setStartVelocity(1.5), trajectories)
        );
    }

    @Override
    public void onSelection() {
        Chassis.getInstance().updateTrajectory(trajectories.size() > 0 ? trajectories : null);
    }
}