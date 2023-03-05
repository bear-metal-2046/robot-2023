package org.tahomarobotics.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.tahomarobotics.robot.arm.ArmMoveCommand;
import org.tahomarobotics.robot.arm.ArmMovements;
import org.tahomarobotics.robot.grabber.ScoreComand;

public class Place extends SequentialCommandGroup {
    public Place(GamePiece piece, Level level) {
        if (level.equals(Level.GROUND)) {
            addCommands(
                    new ScoreComand()
            );
        }
        switch (piece) {
            case CONE -> {
                switch (level) {
                    case HIGH ->
                        addCommands(
                                new ArmMoveCommand(ArmMovements.STOW_TO_HIGH_POLE),
                                new ScoreComand(),
                                new ArmMoveCommand(ArmMovements.HIGH_POLE_TO_STOW)
                        );
                    case MID ->
                        addCommands(
                                new ArmMoveCommand(ArmMovements.STOW_TO_MID_POLE),
                                new ScoreComand(),
                                new ArmMoveCommand(ArmMovements.MID_POLE_TO_STOW)
                        );
                }
            }
            case CUBE -> {
                switch (level) {
                    case HIGH ->
                        addCommands(
                                new ArmMoveCommand(ArmMovements.STOW_TO_HIGH_BOX),
                                new ScoreComand(),
                                new ArmMoveCommand(ArmMovements.HIGH_BOX_TO_STOW)
                        );
                    case MID ->
                        addCommands(
                                new ArmMoveCommand(ArmMovements.STOW_TO_MID_BOX),
                                new ScoreComand(),
                                new ArmMoveCommand(ArmMovements.MID_BOX_TO_STOW)
                        );
                }
            }
        }
    }
}
