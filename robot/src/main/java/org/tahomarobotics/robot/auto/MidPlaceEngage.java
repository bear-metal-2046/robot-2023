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
import org.tahomarobotics.robot.grabber.ScoreCommand;

import java.util.List;

public class MidPlaceEngage extends AutonomousBase {

    private static final Pose2d FIRST_PLACE =
            new Pose2d(Units.inchesToMeters(69.6), Units.inchesToMeters(130.325), new Rotation2d(0));

    private static final Pose2d ENGAGE =
            new Pose2d(Units.inchesToMeters(69.6 + 84.1), Units.inchesToMeters(130.325), new Rotation2d(0));

    private static final Pose2d TAXI =
            new Pose2d(Units.inchesToMeters(69.6 + 172.9), Units.inchesToMeters(130.325), new Rotation2d(0));

    private static final Rotation2d PLACE_HEADING = new Rotation2d(Units.degreesToRadians(180d));

    private static final TrajectoryConfig FWD_CONFIG =
            new TrajectoryConfig(1, 2)
            .setKinematics(Chassis.getInstance().getSwerveDriveKinematics());

    private static final TrajectoryConfig REV_CONFIG =
            new TrajectoryConfig(1, 2)
            .setKinematics(Chassis.getInstance().getSwerveDriveKinematics()).setReversed(true);


    public MidPlaceEngage(DriverStation.Alliance alliance){

        // alliance converted start pose
        super(alliance, new Pose2d(FIRST_PLACE.getTranslation(), PLACE_HEADING));

        // alliance converted trajectories
        Trajectory place2Engage = createTrajectory(FIRST_PLACE, ENGAGE, FWD_CONFIG);
        Trajectory engage2Taxi = createTrajectory(ENGAGE, TAXI, FWD_CONFIG);
        Trajectory taxi2Engage = createTrajectory(TAXI, ENGAGE, REV_CONFIG);

        // alliance converted rotations
        Rotation2d placeHeading = createRotation(PLACE_HEADING);

        addCommands(

                new InstantCommand(() -> Chassis.getInstance().resetOdometry(startPose)),

                new ArmMoveCommand(ArmMovements.STOW_TO_HIGH_POLE),
                new ScoreCommand(0.25),

                new ParallelCommandGroup(
                        new TrajectoryCommand("Mid-to-Engage", place2Engage, placeHeading),
                        new ArmMoveCommand(ArmMovements.HIGH_POLE_TO_STOW)
                ),

                new TrajectoryCommand("Engage-to-Taxi", engage2Taxi, placeHeading),

                new TrajectoryCommand("Back-to-Engage", taxi2Engage, placeHeading),

                new BalancedCommand()
        );
    }
}