package org.tahomarobotics.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.DriverStation;

public enum BlueSideAuto {
    RIGHT_TWO_GAME_PIECE(config -> {
        Pose2d startPose = StartPose.LEFT.getPose(DriverStation.Alliance.Blue);
        Path path = new Path("RIGHT TWO GAME PIECE", config)
                .driveTo(startPose)
                .segmentEnd();

        return  path
                .driveForward(1.5)
                .driveForward(.5).withRotateBy(Math.PI / 2)
                .waitCommand(5)
                .driveForward(1)
                .driveTo(startPose);
    }
    ),

    FORWARD_THEN_NOT(config ->
        new Path("FORWARD THEN NOT", config)
                .driveForward(2)
                .waitCommand(5)
                .driveBackwards(2)
    );

    private final AutoCommand.Supplier command;
    BlueSideAuto(AutoCommand.Supplier command) {
        this.command = command;
    }

    public Path get(TrajectoryConfig config) {
        return command.get(config);
    }
}
