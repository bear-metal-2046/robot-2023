package org.tahomarobotics.robot.auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PlaceCollectEngage extends SequentialCommandGroup {

    public PlaceCollectEngage(DriverStation.Alliance alliance, GamePiece place, Level level, Trajectory trajectory, GamePiece collect) {

        addCommands(

        );
    }
}
