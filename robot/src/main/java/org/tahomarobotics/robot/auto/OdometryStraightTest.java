package org.tahomarobotics.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.tahomarobotics.robot.chassis.Chassis;

import java.util.Collections;
import java.util.List;

public class OdometryStraightTest extends AutonomousBase {

    private static final TrajectoryConfig CONFIG =
        new TrajectoryConfig(2.5, 3)
            .setKinematics(Chassis.getInstance().getSwerveDriveKinematics());

    private static final Pose2d END = new Pose2d(
        Units.feetToMeters(14), 0,
        new Rotation2d()
    );

    public OdometryStraightTest() {
        super(DriverStation.Alliance.Blue, new Pose2d());

        Trajectory trajectory = createTrajectory(new Pose2d(), Collections.emptyList(), END, CONFIG);
        addCommands(
            new InstantCommand(() -> Chassis.getInstance().resetOdometry(startPose)),
            new TrajectoryCommand("Odometry Test", trajectory, new Rotation2d(-Math.PI / 2))
        );
    }
}
