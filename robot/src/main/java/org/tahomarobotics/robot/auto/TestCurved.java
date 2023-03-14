package org.tahomarobotics.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.tahomarobotics.robot.chassis.Chassis;

import java.util.ArrayList;
import java.util.List;

public class TestCurved extends SequentialCommandGroup implements AutonomousCommandIF {
    private Pose2d startPose = new Pose2d(new Translation2d(0, 0), new Rotation2d(0));
    private Pose2d endPose = new Pose2d(new Translation2d(Units.inchesToMeters(192.5), Units.inchesToMeters(192.5)), new Rotation2d(Math.PI / 2));
    private List<Trajectory> poses = new ArrayList<>();

    public TestCurved() {
        TrajectoryConfig config = new TrajectoryConfig(2.5, 5)
                .setKinematics(Chassis.getInstance().getSwerveDriveKinematics());
        addCommands(
                new InstantCommand(() -> Chassis.getInstance().resetOdometry(startPose)),
                Drive.drive("Test-Curve", startPose, endPose, new Rotation2d(0), config, poses)
        );
    }

    @Override
    public List<Trajectory> getTrajectories() {
        return poses;
    }

    @Override
    public Pose2d getStartPose() {
        return startPose;
    }
}
