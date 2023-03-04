package org.tahomarobotics.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Place extends SequentialCommandGroup {
    public Place(GamePiece piece, Level level) {
        switch (level) {
            case HIGH -> {
                addCommands();
            }
            case MID -> {
                addCommands();
            }
            case GROUND -> {
                addCommands();
            }
        }
    }
}
