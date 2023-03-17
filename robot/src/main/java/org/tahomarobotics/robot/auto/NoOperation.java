package org.tahomarobotics.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.List;

public class NoOperation extends SequentialCommandGroup implements AutonomousCommandIF {

    private static final Pose2d blueCorner = new Pose2d(0,0,new Rotation2d(0));
    private static final Pose2d redCorner = new Pose2d(Units.inchesToMeters(651.25),Units.inchesToMeters(315.5), new Rotation2d(Math.PI));

    private final Pose2d startPose;

    public NoOperation(DriverStation.Alliance alliance) {
        startPose = alliance == DriverStation.Alliance.Blue ? blueCorner : redCorner;
    }

    @Override
    public List<Trajectory> getTrajectories() {
        return null;
    }

    @Override
    public Pose2d getStartPose() {
        return startPose;
    }
}
