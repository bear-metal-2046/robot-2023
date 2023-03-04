package org.tahomarobotics.robot.auto;

import edu.wpi.first.math.trajectory.Trajectory;

public class PlaceCollect extends PlaceTaxi {
    public PlaceCollect(GamePiece place, Level level, Trajectory trajectory, GamePiece collect) {
        super(place, level, trajectory);
        addCommands(
            //... TODO
            new TrajectoryCommand(trajectory)
            //... TODO
        );
    }
}
