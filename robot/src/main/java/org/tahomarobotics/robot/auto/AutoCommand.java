package org.tahomarobotics.robot.auto;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public abstract class AutoCommand extends SequentialCommandGroup implements AutonomousCommandIF {
    public interface Supplier {
        Path get(TrajectoryConfig config);
    }
}
