package org.tahomarobotics.robot.auto;

import edu.wpi.first.math.trajectory.Trajectory;

public class PlaceCollectEngage extends PlaceCollect {

    public PlaceCollectEngage(GamePiece place, Level level, Trajectory trajectory, GamePiece collect) {
        super(place, level, trajectory, collect);
        addCommands(

        );
    }
}
