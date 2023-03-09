package org.tahomarobotics.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.tahomarobotics.robot.arm.ArmMoveCommand;
import org.tahomarobotics.robot.arm.ArmMovements;
import org.tahomarobotics.robot.chassis.Chassis;

import java.util.ArrayList;
import java.util.List;

public class PlaceTaxi extends Place implements AutonomousCommandIF {

    private static final List<Trajectory> trajectories = new ArrayList<>();

    @Override
    public List<Trajectory> getTrajectories() {
        return trajectories;
    }
    private final Pose2d startPose = new Pose2d(Units.inchesToMeters(582.9), Units.inchesToMeters(220.1),
            new Rotation2d(Units.degreesToRadians(180)));
    private final Pose2d firstPreCollect = new Pose2d(Units.inchesToMeters(434.2), Units.inchesToMeters(201.8),
            new Rotation2d(Units.degreesToRadians(180)));
    private final Rotation2d fistCollectRot = new Rotation2d(Units.degreesToRadians(180));
    public PlaceTaxi(GamePiece piece, Level level) {
        super(piece, level);
        TrajectoryConfig config = new TrajectoryConfig(2, 3)
                .setKinematics(Chassis.getInstance().getSwerveDriveKinematics());
        addCommands(
                new InstantCommand(() -> Chassis.getInstance().resetOdometry(new Pose2d(startPose.getTranslation(), new Rotation2d(0)))),
                new ParallelCommandGroup(
                        Drive.drive(startPose, firstPreCollect, fistCollectRot, config, trajectories),
                        new ArmMoveCommand(ArmMovements.HIGH_POLE_TO_STOW)
                )
        );
    }
}