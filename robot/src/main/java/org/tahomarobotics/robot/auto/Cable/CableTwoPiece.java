package org.tahomarobotics.robot.auto.Cable;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import org.tahomarobotics.robot.arm.ArmMoveCommand;
import org.tahomarobotics.robot.arm.ArmMovements;
import org.tahomarobotics.robot.auto.AutonomousBase;
import org.tahomarobotics.robot.auto.BalancedCommand;
import org.tahomarobotics.robot.auto.TrajectoryCommand;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.grabber.IngestCommand;
import org.tahomarobotics.robot.grabber.ScoreCommand;

import java.util.List;

public class CableTwoPiece extends AutonomousBase {
    private static final Pose2d START_POSE = new Pose2d(
            Units.inchesToMeters(70.563), Units.inchesToMeters(20.029),
            new Rotation2d(0)
    );

    private static final Pose2d FIRST_COLLECT = new Pose2d(
            Units.inchesToMeters(280.035), Units.inchesToMeters(39.211),
            new Rotation2d(0)
    );
    private static final Pose2d FIRST_COLLECT_MIRRORED = new Pose2d(
            Units.inchesToMeters(280.035), Units.inchesToMeters(39.211),
            new Rotation2d(Math.PI)
    );

    private static final Pose2d BUMP_GOING_REV = new Pose2d(
            Units.inchesToMeters(133.10), Units.inchesToMeters(24.029),
            new Rotation2d(Math.PI)
    );

    private static final Pose2d SECOND_PLACE = new Pose2d(
            Units.inchesToMeters(68.563), Units.inchesToMeters(42.029 + 8),
            new Rotation2d(Math.PI)
    );

    private static final Rotation2d PLACE_HEADING = new Rotation2d(Math.PI);
    private static final Rotation2d COLLECT_HEADING = new Rotation2d(Units.degreesToRadians(0));

    private static final TrajectoryConfig CONFIG = new TrajectoryConfig(2, 3)
            .setKinematics(Chassis.getInstance().getSwerveDriveKinematics());

    public CableTwoPiece(DriverStation.Alliance alliance) {
        super(alliance, new Pose2d(START_POSE.getTranslation(), PLACE_HEADING));

        Trajectory oneForAll = createTrajectory(START_POSE, FIRST_COLLECT, CONFIG);
        Trajectory allForOne = createTrajectory(FIRST_COLLECT_MIRRORED, List.of(BUMP_GOING_REV.getTranslation()), SECOND_PLACE, CONFIG);

        Rotation2d collectHeading = createRotation(COLLECT_HEADING);
        Rotation2d placeHeading = createRotation(PLACE_HEADING);

        TrajectoryCommand.TurnDirection turnDirection1 = alliance == DriverStation.Alliance.Blue ?
                TrajectoryCommand.TurnDirection.CLOCKWISE : TrajectoryCommand.TurnDirection.COUNTER_CLOCKWISE;

        TrajectoryCommand.TurnDirection turnDirection2 = alliance == DriverStation.Alliance.Blue ?
                TrajectoryCommand.TurnDirection.COUNTER_CLOCKWISE : TrajectoryCommand.TurnDirection.CLOCKWISE;

        addCommands(
                new InstantCommand(() -> Chassis.getInstance().resetOdometry(startPose)),
                new ArmMoveCommand(ArmMovements.START_TO_HIGH_POLE),
                new ScoreCommand(0.25),
                new ParallelCommandGroup(
                        new TrajectoryCommand("Start to Collect", oneForAll, collectHeading, 0.4, 0.8,
                                turnDirection1),
                        new SequentialCommandGroup(
                                new ArmMoveCommand(ArmMovements.HIGH_POLE_TO_STOW),
                                new WaitCommand(0.25),
                                new ParallelCommandGroup(
                                        new ArmMoveCommand(ArmMovements.STOW_TO_CUBE_COLLECT),
                                        new IngestCommand(1.5)
                                )
                        )
                ),
                new ParallelCommandGroup(
                        new TrajectoryCommand("Collect to Second Place", allForOne, placeHeading, 0.0, 0.5,
                                turnDirection2),
                        new SequentialCommandGroup(
                                new ArmMoveCommand(ArmMovements.CUBE_COLLECT_TO_STOW),
                                new WaitCommand(1),
                                new ArmMoveCommand(ArmMovements.STOW_TO_HIGH_BOX)
                        )
                ),
                new ScoreCommand(0.25),
                new ArmMoveCommand(ArmMovements.HIGH_BOX_TO_STOW)
        );
    }
}
