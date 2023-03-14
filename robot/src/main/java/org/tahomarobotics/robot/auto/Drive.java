package org.tahomarobotics.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

import java.util.List;

public class Drive {
    public static TrajectoryCommand drive(String name, Pose2d start, Pose2d end, Rotation2d endRot, TrajectoryConfig config,
                                   List<Trajectory> out) {
        return drive(name, start, List.of(), end, endRot, config, out);
    }
    public static TrajectoryCommand drive(String name, Pose2d start, List<Translation2d> waypoints, Pose2d end, Rotation2d endRot, TrajectoryConfig config,
                                   List<Trajectory> out) {
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                start,
                waypoints,
                end,
                config
        );

        out.add(trajectory);
        return new TrajectoryCommand(name, trajectory, endRot, config.getEndVelocity() > 0);
    }

    public static TrajectoryCommand drive(String name, Trajectory trajectory, Rotation2d endRot, List<Trajectory> out) {
        out.add(trajectory);
        boolean hasEndVelocity = trajectory.sample(trajectory.getTotalTimeSeconds()).velocityMetersPerSecond > 0;
        return new TrajectoryCommand(name, trajectory, endRot, hasEndVelocity);
    }
}
