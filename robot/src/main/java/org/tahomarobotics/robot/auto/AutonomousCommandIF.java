package org.tahomarobotics.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import org.tahomarobotics.robot.chassis.Chassis;

import java.util.List;

public interface AutonomousCommandIF {

    List<Trajectory> getTrajectories();


    default void onSelection() {
        var chassis = Chassis.getInstance();
        var trajectories = getTrajectories();

        chassis.updateTrajectory(trajectories.size() > 0 ? trajectories : null);
        chassis.resetOdometry(trajectories.size() > 0 ? trajectories.get(0).getInitialPose() : new Pose2d());
    }
}
