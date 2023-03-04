package org.tahomarobotics.robot.auto;

import edu.wpi.first.math.trajectory.Trajectory;

public class PlaceCollectPlace extends PlaceCollect {

    public PlaceCollectPlace(GamePiece place, Level level, Trajectory trajectory,
                             GamePiece collect,
                             Trajectory backTrajectory, GamePiece backPlace, Level backLevel) {
        super(place, level, trajectory, collect);
        addCommands(
                new TrajectoryCommand(backTrajectory),
                new Place(backPlace, backLevel)
        );
    }
}
