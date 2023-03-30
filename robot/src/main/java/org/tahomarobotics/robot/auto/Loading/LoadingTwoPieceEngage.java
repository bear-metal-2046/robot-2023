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
import org.tahomarobotics.robot.arm.ArmMoveCommand;
import org.tahomarobotics.robot.arm.ArmMovements;
import org.tahomarobotics.robot.auto.AutonomousBase;
import org.tahomarobotics.robot.auto.BalancedCommand;
import org.tahomarobotics.robot.auto.TrajectoryCommand;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.grabber.IngestCommand;
import org.tahomarobotics.robot.grabber.ScoreCommand;

import java.util.List;

public class LoadingTwoPieceEngage extends AutonomousBase {
    private static final Pose2d FIRST_PLACE = new Pose2d(Units.inchesToMeters(69.6), Units.inchesToMeters(196.325),
            new Rotation2d(0));
    private static final Pose2d SECOND_PLACE = new Pose2d(Units.inchesToMeters(69.6), Units.inchesToMeters(196.325 - 32.0),
            new Rotation2d(Units.degreesToRadians(180)));
    private static final Pose2d SECOND_PLACE_PT_2 = new Pose2d(Units.inchesToMeters(69.6), Units.inchesToMeters(196.325 - 32.0),
            new Rotation2d(Units.degreesToRadians(0)));

    //Collect Points
    private static final Pose2d FIRST_COLLECT = new Pose2d(Units.inchesToMeters(69.6 + 210.9), Units.inchesToMeters(196.325 - 20),
            new Rotation2d(Units.degreesToRadians(-35)));
    private static final Pose2d FIRST_COLLECT_PT_2 = new Pose2d(Units.inchesToMeters(69.6 + 210.9), Units.inchesToMeters(196.325 - 20),
            new Rotation2d(Units.degreesToRadians(180)));

    private static final Pose2d ENGAGE = new Pose2d(Units.inchesToMeters(69.6 + 110.0), Units.inchesToMeters(125.0),
            new Rotation2d(Units.degreesToRadians(0)));

    //Mid-Translations
    private static final Translation2d MID_PT = new Translation2d(Units.inchesToMeters(69.6 + 84.0), Units.inchesToMeters(196.325 - 15.0));
    private static final Translation2d MID_PT_2 = new Translation2d(Units.inchesToMeters(69.6 + 80.0), Units.inchesToMeters(196.325 - 24));
    private static final Translation2d MID_PT_3 = new Translation2d(Units.inchesToMeters(69.6 + 12.0), Units.inchesToMeters(125.0));

    private static final Rotation2d PLACE_HEADING = new Rotation2d(Units.degreesToRadians(180));
    private static final Rotation2d COLLECT_HEADING = new Rotation2d(Units.degreesToRadians(-35));


    private static final TrajectoryConfig CONFIG = new TrajectoryConfig(2.0, 3)
            .setKinematics(Chassis.getInstance().getSwerveDriveKinematics());

    public LoadingTwoPieceEngage(DriverStation.Alliance alliance) {
        // alliance converted start pose
        super(alliance, new Pose2d(FIRST_PLACE.getTranslation(), PLACE_HEADING));

        // alliance converted trajectories
        Trajectory collectTrajectory = createTrajectory(FIRST_PLACE, List.of(MID_PT), FIRST_COLLECT, CONFIG);
        Trajectory placeTrajectory = createTrajectory(FIRST_COLLECT_PT_2, List.of(MID_PT_2), SECOND_PLACE, CONFIG);
        Trajectory engageTrajectory = createTrajectory(SECOND_PLACE_PT_2, List.of(MID_PT_3), ENGAGE, CONFIG);

        // alliance converted rotations
        Rotation2d collectHeading = createRotation(COLLECT_HEADING);
        Rotation2d placeHeading = createRotation(PLACE_HEADING);

        addCommands(
                new InstantCommand(() -> Chassis.getInstance().resetOdometry(startPose)),
                new ArmMoveCommand(ArmMovements.START_TO_HIGH_POLE),
                new ScoreCommand(0.25),
                new ParallelCommandGroup(
                        new TrajectoryCommand("Start to collect", collectTrajectory, collectHeading, 0.3, 0.7,
                                alliance == DriverStation.Alliance.Blue ? TrajectoryCommand.TurnDirection.COUNTER_CLOCKWISE : TrajectoryCommand.TurnDirection.CLOCKWISE),
                        new SequentialCommandGroup(
                                new ArmMoveCommand(ArmMovements.HIGH_POLE_TO_STOW),
                                new ParallelCommandGroup(
                                        new ArmMoveCommand(ArmMovements.STOW_TO_CUBE_COLLECT),
                                        new IngestCommand(1.5)
                                )
                        )
                ),
                new ParallelCommandGroup(
                        new TrajectoryCommand("Collect to Place", placeTrajectory, placeHeading, 0.0, 0.5,
                                alliance == DriverStation.Alliance.Blue ? TrajectoryCommand.TurnDirection.CLOCKWISE : TrajectoryCommand.TurnDirection.COUNTER_CLOCKWISE),
                        new SequentialCommandGroup(
                                new ArmMoveCommand(ArmMovements.CUBE_COLLECT_TO_STOW),
                                new WaitCommand(0.75),
                                new ArmMoveCommand(ArmMovements.STOW_TO_HIGH_BOX),
                                new ScoreCommand(0.15)
                        )
                ),
                new ParallelCommandGroup(
                        new TrajectoryCommand("Place to Engage", engageTrajectory, placeHeading),
                        new ArmMoveCommand(ArmMovements.HIGH_BOX_TO_STOW)
                ),
                new BalancedCommand()
        );
    }
}
