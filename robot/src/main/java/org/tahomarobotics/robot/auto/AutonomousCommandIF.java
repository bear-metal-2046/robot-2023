package org.tahomarobotics.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import org.tahomarobotics.robot.chassis.Chassis;

import java.util.List;

public interface AutonomousCommandIF {

    List<Trajectory> getTrajectories();

    Pose2d getStartPose();

    default void onSelection() {
        var chassis = Chassis.getInstance();

        chassis.updateTrajectory(getTrajectories());
        chassis.resetOdometry(getStartPose());
    }
}
