package org.tahomarobotics.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;

public enum NoOperation {
    NO_OP(config -> new Path("NO_OPERATION", config));

    private final AutoCommand.Supplier command;
    NoOperation(AutoCommand.Supplier command) {
        this.command = command;
    }

    public Path get(TrajectoryConfig config) {
        return command.get(config);
    }
}
