package org.tahomarobotics.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.tahomarobotics.robot.arm.ArmMoveCommand;
import org.tahomarobotics.robot.arm.ArmMovements;
import org.tahomarobotics.robot.grabber.ScoreCommand;

public class Place extends SequentialCommandGroup {
    public Place(GamePiece piece, Level level) {
        if (level.equals(Level.GROUND)) {
            addCommands(
                    new ScoreCommand(0.25)
            );
        }
        switch (piece) {
            case CONE -> {
                switch (level) {
                    case HIGH ->
                            //TODO What ever I do to this, do to all
                        addCommands(
                                new ArmMoveCommand(ArmMovements.STOW_TO_HIGH_POLE),
                                new ScoreCommand(0.25)
                        );
                    case MID ->
                        addCommands(
                                new ArmMoveCommand(ArmMovements.STOW_TO_MID_POLE),
                                new ScoreCommand(0.25)
                        );
                }
            }
            case CUBE -> {
                switch (level) {
                    case HIGH ->
                        addCommands(
                                new ArmMoveCommand(ArmMovements.STOW_TO_HIGH_BOX),
                                new ScoreCommand(0.25)
                        );
                    case MID ->
                        addCommands(
                                new ArmMoveCommand(ArmMovements.STOW_TO_MID_BOX),
                                new ScoreCommand(0.25)
                        );
                }
            }
        }
    }
}
