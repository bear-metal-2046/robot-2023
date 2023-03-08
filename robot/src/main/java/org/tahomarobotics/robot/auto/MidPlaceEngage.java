package org.tahomarobotics.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.tahomarobotics.robot.arm.ArmMoveCommand;
import org.tahomarobotics.robot.arm.ArmMovements;
import org.tahomarobotics.robot.chassis.Chassis;

import java.util.ArrayList;
import java.util.List;

public class MidPlaceEngage extends Place implements AutonomousCommandIF{

    private List<Trajectory> list = new ArrayList<>();

    private final Pose2d startPose = new Pose2d(Units.inchesToMeters(582.9), Units.inchesToMeters(152.2),
            new Rotation2d(Units.degreesToRadians(180)));
    private final Pose2d taxi = new Pose2d(Units.inchesToMeters(420.9), Units.inchesToMeters(152.2),
            new Rotation2d(Units.degreesToRadians(180)));
    private final Pose2d engage = new Pose2d(Units.inchesToMeters(498.8), Units.inchesToMeters(152.2),
            new Rotation2d(Units.degreesToRadians(180)));

    private final Rotation2d rot = new Rotation2d(Units.degreesToRadians(0));

    public MidPlaceEngage(GamePiece piece, Level level){
        super(piece, level);

        TrajectoryConfig config = new TrajectoryConfig(1, 2)
                .setKinematics(Chassis.getInstance().getSwerveDriveKinematics());
        TrajectoryConfig reversedConfig = new TrajectoryConfig(1, 2)
                .setKinematics(Chassis.getInstance().getSwerveDriveKinematics()).setReversed(true);

        addCommands(
                new InstantCommand(() -> Chassis.getInstance().resetOdometry(new Pose2d(startPose.getTranslation(), new Rotation2d(0)))),
                new ParallelCommandGroup(
                        Drive.drive(startPose, engage, rot, config, list),
                        new ArmMoveCommand(ArmMovements.HIGH_POLE_TO_STOW)
                ),
                Drive.drive(engage, taxi, rot, config, list),
                Drive.drive(taxi, engage, rot, reversedConfig, list),
                new BalancedCommand()
        );
    }

    @Override
    public void onSelection() {
        Chassis.getInstance().updateTrajectory(list.size() > 0 ? list : null);
    }
}