package org.tahomarobotics.robot.auto;

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
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.grabber.IngestCommand;
import org.tahomarobotics.robot.grabber.ScoreCommand;

import java.util.List;

public class PlaceCollectPlace extends AutonomousBase {

    private static final Pose2d FIRST_PLACE = new Pose2d(Units.inchesToMeters(69.6), Units.inchesToMeters(196.325),
            new Rotation2d(0));
    private static final Pose2d SECOND_PLACE = new Pose2d(Units.inchesToMeters(69.6), Units.inchesToMeters(196.325 - 10.0),
            new Rotation2d(Units.degreesToRadians(180)));
    private static final Translation2d MID_PT = new Translation2d(Units.inchesToMeters(69.6 + 84.0), Units.inchesToMeters(196.325 - 8.0));
    private static final Translation2d MID_PT_2 = new Translation2d(Units.inchesToMeters(69.6 + 40.0), Units.inchesToMeters(196.325 - 8.0));
    private static final Pose2d FIRST_COLLECT = new Pose2d(Units.inchesToMeters(69.6 + 195.9), Units.inchesToMeters(196.325 - 5),
            new Rotation2d(Units.degreesToRadians(-35)));
    private static final Pose2d FIRST_COLLECT_2 = new Pose2d(Units.inchesToMeters(69.6 + 195.9), Units.inchesToMeters(196.325 - 5),
            new Rotation2d(Units.degreesToRadians(180)));
    private static final Rotation2d PLACE_HEADING = new Rotation2d(Units.degreesToRadians(180));
    private static final Rotation2d COLLECT_HEADING = new Rotation2d(Units.degreesToRadians(-35));

    private static final TrajectoryConfig CONFIG = new TrajectoryConfig(2.0, 3)
            .setKinematics(Chassis.getInstance().getSwerveDriveKinematics());
    private static final TrajectoryConfig REVERSED_CONFIG = new TrajectoryConfig(2.0, 3)
            .setKinematics(Chassis.getInstance().getSwerveDriveKinematics()).setReversed(true);


    public PlaceCollectPlace(DriverStation.Alliance alliance) {

        // alliance converted start pose
        super(alliance, new Pose2d(FIRST_PLACE.getTranslation(), PLACE_HEADING));

        // alliance converted trajectories
        Trajectory collectTrajectory = createTrajectory(FIRST_PLACE, List.of(MID_PT), FIRST_COLLECT, CONFIG);
        Trajectory placeTrajectory = createTrajectory(FIRST_COLLECT_2, List.of(MID_PT_2), SECOND_PLACE, CONFIG);

        // alliance converted rotations
        Rotation2d collectHeading = createRotation(COLLECT_HEADING);
        Rotation2d placeHeading = createRotation(PLACE_HEADING);

        addCommands(
                new InstantCommand(() -> Chassis.getInstance().resetOdometry(startPose)),
                new ArmMoveCommand(ArmMovements.START_TO_HIGH_POLE),
                new ScoreCommand(0.25),
                new ParallelCommandGroup(
                        new TrajectoryCommand("Start to collect", collectTrajectory, collectHeading, 0.3, 0.8),
                        new SequentialCommandGroup(
                                new ArmMoveCommand(ArmMovements.HIGH_POLE_TO_STOW),
                                new ParallelCommandGroup(
                                        new ArmMoveCommand(ArmMovements.STOW_TO_CUBE_COLLECT),
                                        new IngestCommand(2)
                                )
                        )
                ),
                new ParallelCommandGroup(
                        new TrajectoryCommand("Collect to Place", placeTrajectory, placeHeading, 0.2, 0.7),
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
