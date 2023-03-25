package org.tahomarobotics.robot.auto.Cable;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.tahomarobotics.robot.arm.ArmMoveCommand;
import org.tahomarobotics.robot.arm.ArmMovements;
import org.tahomarobotics.robot.auto.AutonomousBase;
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
            Units.inchesToMeters(276.035), Units.inchesToMeters(36.211),
            new Rotation2d(0)
    );
    private static final Pose2d FIRST_COLLECT_MIRRORED = new Pose2d(
            Units.inchesToMeters(276.035), Units.inchesToMeters(36.211),
            new Rotation2d(Math.PI)
    );

    private static final Pose2d BUMP_GOING = new Pose2d(
            Units.inchesToMeters(133.10), Units.inchesToMeters(21.029),
            new Rotation2d(0)
    );

    private static final Pose2d BUMP_COMING = new Pose2d(
            Units.inchesToMeters(173.10), Units.inchesToMeters(24.029),
            new Rotation2d(0)
    );

    private static final Pose2d BUMP_GOING_REV = new Pose2d(
            Units.inchesToMeters(133.10), Units.inchesToMeters(24.029),
            new Rotation2d(Math.PI)
    );

    private static final Pose2d BUMP_COMING_REV = new Pose2d(
            Units.inchesToMeters(173.10), Units.inchesToMeters(21.029),
            new Rotation2d(Math.PI)
    );

    private static final Pose2d SECOND_PLACE = new Pose2d(
            Units.inchesToMeters(70.563), Units.inchesToMeters(42.029),
            new Rotation2d(Math.PI)
    );

    private static final Rotation2d PLACE_HEADING = new Rotation2d(Math.PI);
    private static final Rotation2d COLLECT_HEADING = new Rotation2d(Units.degreesToRadians(10));

    private static final TrajectoryConfig CONFIG_1 = new TrajectoryConfig(2.0, 3)
            .setKinematics(Chassis.getInstance().getSwerveDriveKinematics()).setEndVelocity(0.75);
    private static final TrajectoryConfig CONFIG_2 = new TrajectoryConfig(2, 1)
            .setKinematics(Chassis.getInstance().getSwerveDriveKinematics());
    private static final TrajectoryConfig CONFIG_3 = new TrajectoryConfig(2, 3)
            .setKinematics(Chassis.getInstance().getSwerveDriveKinematics()).setStartVelocity(0.75);

    public CableTwoPiece(DriverStation.Alliance alliance) {
        super(alliance, new Pose2d(START_POSE.getTranslation(), PLACE_HEADING));

//        Trajectory startToBump = createTrajectory(START_POSE, BUMP_GOING, CONFIG_1);
//        Trajectory hump = createTrajectory(BUMP_GOING, BUMP_COMING, CONFIG_2);
//        Trajectory bumpToCollect = createTrajectory(BUMP_COMING, FIRST_COLLECT, CONFIG_3);

        Trajectory oneForAll = createTrajectory(START_POSE, FIRST_COLLECT, CONFIG_2);
        Trajectory allForOne = createTrajectory(FIRST_COLLECT_MIRRORED, List.of(BUMP_GOING_REV.getTranslation()), SECOND_PLACE, CONFIG_2);

//        Trajectory collectToBump = createTrajectory(FIRST_COLLECT_MIRRORED, BUMP_COMING_REV, CONFIG_1);
//        Trajectory humpReverse = createTrajectory(BUMP_COMING_REV, BUMP_GOING_REV, CONFIG_2);
//        Trajectory bumpToPlace = createTrajectory(BUMP_GOING_REV, SECOND_PLACE, CONFIG_3);

        Rotation2d collectHeading = createRotation(COLLECT_HEADING);
        Rotation2d placeHeading = createRotation(PLACE_HEADING);

        addCommands(
                new InstantCommand(() -> Chassis.getInstance().resetOdometry(startPose)),
                new ArmMoveCommand(ArmMovements.START_TO_HIGH_POLE),
                new ScoreCommand(0.25),
                new ParallelCommandGroup(
                        new TrajectoryCommand("Start to Collect", oneForAll, collectHeading, 0.0, 0.8,
                                Math.random() < 0.5 ? TrajectoryCommand.TurnDirection.COUNTER_CLOCKWISE : TrajectoryCommand.TurnDirection.CLOCKWISE),
                        new SequentialCommandGroup(
                                new ArmMoveCommand(ArmMovements.HIGH_POLE_TO_STOW),
                                new WaitCommand(1.0),
                                new ParallelCommandGroup(
                                        new ArmMoveCommand(ArmMovements.STOW_TO_CUBE_COLLECT),
                                        new IngestCommand(2.0)
                                )
                        )
                ),
                new ParallelCommandGroup(
                        new TrajectoryCommand("Collect to Second Place", allForOne, placeHeading, 0, 1,
                                Math.random() < 0.5 ? TrajectoryCommand.TurnDirection.COUNTER_CLOCKWISE : TrajectoryCommand.TurnDirection.CLOCKWISE),
                        new SequentialCommandGroup(
                                new ArmMoveCommand(ArmMovements.CUBE_COLLECT_TO_STOW),
                                new WaitCommand(3),
                                new ArmMoveCommand(ArmMovements.STOW_TO_HIGH_BOX)
                        )
                ),
                new ScoreCommand(0.25),
                new ArmMoveCommand(ArmMovements.HIGH_BOX_TO_STOW)
        );
    }
}
