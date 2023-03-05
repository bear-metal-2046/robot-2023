package org.tahomarobotics.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.tahomarobotics.robot.arm.ArmMoveCommand;
import org.tahomarobotics.robot.arm.ArmMovements;
import org.tahomarobotics.robot.grabber.InjestCommand;

public class Collect extends SequentialCommandGroup {
    public Collect(GamePiece piece) {
        switch (piece) {
            case CONE -> {
                addCommands(
                        new ArmMoveCommand(ArmMovements.STOW_TO_CONE_COLLECT),
                        new InjestCommand(),
                        new ArmMoveCommand(ArmMovements.CONE_COLLECT_TO_STOW)
                );
            }
            case CUBE -> {
                addCommands(
                        new ArmMoveCommand(ArmMovements.STOW_TO_CUBE_COLLECT),
                        new InjestCommand(),
                        new ArmMoveCommand(ArmMovements.CUBE_COLLECT_TO_STOW)
                );
            }
        }
    }
}
