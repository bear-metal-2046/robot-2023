package org.tahomarobotics.robot.auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.tahomarobotics.robot.chassis.Chassis;

import java.util.ArrayList;
import java.util.List;

public class NoOperation extends SequentialCommandGroup implements AutonomousCommandIF {
    private static final List<Trajectory> trajectories = new ArrayList<>();

    @Override
    public List<Trajectory> getTrajectories() {
        return trajectories;
    }

}
