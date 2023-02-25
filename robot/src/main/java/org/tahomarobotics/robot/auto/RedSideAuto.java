package org.tahomarobotics.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;

public enum RedSideAuto {
    DRIVE_2_1_BACK(config ->
            new Path("DRIVE 2 1 BACK", config)
                    .driveForward(1.5)
                    .driveForward(.5).withRotateBy(Math.PI / 2)
                    .waitCommand(5)
                    .driveForward(1)
                    .driveToStart()
    ),

    TURNT(config ->
            new Path("TURNT", config)
                    .driveForward(1.5)
                    .driveForward(.5).withRotateBy(Math.PI / 2)
    );

    private final AutoCommand.Supplier command;
    RedSideAuto(AutoCommand.Supplier command) {
        this.command = command;
    }

    public Path get(TrajectoryConfig config) {
        return command.get(config);
    }
}
