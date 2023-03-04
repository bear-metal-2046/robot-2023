package org.tahomarobotics.robot.auto;

import edu.wpi.first.math.trajectory.Trajectory;

public class PlaceTaxi extends Place {
    public PlaceTaxi(GamePiece piece, Level level, Trajectory trajectory) {
        super(piece, level);
        addCommands(
                new TrajectoryCommand(trajectory)
        );
    }
}
