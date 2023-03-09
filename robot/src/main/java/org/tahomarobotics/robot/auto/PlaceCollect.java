package org.tahomarobotics.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.tahomarobotics.robot.arm.ArmMoveCommand;
import org.tahomarobotics.robot.arm.ArmMovements;
import org.tahomarobotics.robot.chassis.Chassis;

import java.util.ArrayList;
import java.util.List;

public class PlaceCollect extends PlaceTaxi {
    private List<Trajectory> trajectories = new ArrayList<>();
    private final Pose2d firstPreCollect = new Pose2d(Units.inchesToMeters(434.2), Units.inchesToMeters(201.8),
            new Rotation2d(Units.degreesToRadians(180)));
    private final Pose2d firstCollect = new Pose2d(Units.inchesToMeters(399.2), Units.inchesToMeters(201.8),
            new Rotation2d(Units.degreesToRadians(180)));
    private final Rotation2d fistCollectRot = new Rotation2d(Units.degreesToRadians(180));
    public PlaceCollect(GamePiece place, Level level) {
        super(place, level);

        TrajectoryConfig config = new TrajectoryConfig(2, 3)
                .setKinematics(Chassis.getInstance().getSwerveDriveKinematics());

        addCommands(
                new ParallelCommandGroup(
                        new Collect(GamePiece.CUBE),
                        new SequentialCommandGroup(
                                new WaitCommand(0.5),
                                Drive.drive(firstPreCollect, firstCollect, fistCollectRot, config, trajectories)
                        )
                ),
                new ArmMoveCommand(ArmMovements.CUBE_COLLECT_TO_STOW)
        );
    }
}
