package org.tahomarobotics.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Collect extends SequentialCommandGroup {
    public Collect(GamePiece piece) {
        switch (piece) {
            case CONE -> {
                addCommands();
            }
            case CUBE -> {
                addCommands();
            }
        }
    }
}
