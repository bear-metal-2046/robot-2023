/**
 * Copyright 2023 Tahoma Robotics - http://tahomarobotics.org - Bear Metal 2046 FRC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including without
 * limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
 * Software, and to permit persons to whom the Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions
 * of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
 * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 */
package org.tahomarobotics.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.arm.ArmMoveCommand;
import org.tahomarobotics.robot.lights.LED;
import org.tahomarobotics.robot.lights.LEDConstants;

import java.util.HashMap;
import java.util.Map;

import static org.tahomarobotics.robot.arm.ArmMovements.*;

public class OperatorArmMoveSelection {

    private static final Logger logger = LoggerFactory.getLogger(OperatorArmMoveSelection.class);

        public enum ConeOrCube { CONE, CUBE }
    public enum ScoringLevel { HIGH(0), MID(90);
        public final int pov;
        ScoringLevel(int pov) {
            this.pov = pov;
        }
    }
    public enum CollectLevel {FEEDER, LOW }
    public enum ArmPosition { SCORE, STOW, COLLECT }
    private record ScoreCommandKey(ScoringLevel level, ConeOrCube mode, ArmPosition armPosition) {}
    private record CollectCommandKey(CollectLevel level, ConeOrCube mode, ArmPosition armPosition) {}
    private final Map<ScoreCommandKey, ArmMove> scoreCommands = new HashMap<>();
    private final Map<CollectCommandKey, ArmMove> collectCommands = new HashMap<>();

    public OperatorArmMoveSelection() {
        scoreCommands.put(new ScoreCommandKey(ScoringLevel.HIGH, ConeOrCube.CUBE, ArmPosition.SCORE), STOW_TO_HIGH_BOX);
        scoreCommands.put(new ScoreCommandKey(ScoringLevel.HIGH, ConeOrCube.CUBE, ArmPosition.STOW), HIGH_BOX_TO_STOW);
        scoreCommands.put(new ScoreCommandKey(ScoringLevel.HIGH, ConeOrCube.CONE, ArmPosition.SCORE), STOW_TO_HIGH_POLE);
        scoreCommands.put(new ScoreCommandKey(ScoringLevel.HIGH, ConeOrCube.CONE, ArmPosition.STOW), HIGH_POLE_TO_STOW);

        scoreCommands.put(new ScoreCommandKey(ScoringLevel.MID, ConeOrCube.CUBE, ArmPosition.SCORE), STOW_TO_MID_BOX);
        scoreCommands.put(new ScoreCommandKey(ScoringLevel.MID, ConeOrCube.CUBE, ArmPosition.STOW), MID_BOX_TO_STOW);
        scoreCommands.put(new ScoreCommandKey(ScoringLevel.MID, ConeOrCube.CONE, ArmPosition.SCORE), STOW_TO_MID_POLE);
        scoreCommands.put(new ScoreCommandKey(ScoringLevel.MID, ConeOrCube.CONE, ArmPosition.STOW), MID_POLE_TO_STOW);

        collectCommands.put(new CollectCommandKey(CollectLevel.FEEDER, ConeOrCube.CONE, ArmPosition.COLLECT), STOW_TO_CONE_FEEDER_COLLECT);
        collectCommands.put(new CollectCommandKey(CollectLevel.FEEDER, ConeOrCube.CONE, ArmPosition.STOW), CONE_FEEDER_COLLECT_TO_STOW);
        collectCommands.put(new CollectCommandKey(CollectLevel.FEEDER, ConeOrCube.CUBE, ArmPosition.COLLECT), STOW_TO_CUBE_FEEDER_COLLECT);
        collectCommands.put(new CollectCommandKey(CollectLevel.FEEDER, ConeOrCube.CUBE, ArmPosition.STOW), CUBE_FEEDER_COLLECT_TO_STOW);

        collectCommands.put(new CollectCommandKey(CollectLevel.LOW, ConeOrCube.CONE, ArmPosition.COLLECT), STOW_TO_CONE_COLLECT);
        collectCommands.put(new CollectCommandKey(CollectLevel.LOW, ConeOrCube.CONE, ArmPosition.STOW), CONE_COLLECT_TO_STOW);
        collectCommands.put(new CollectCommandKey(CollectLevel.LOW, ConeOrCube.CUBE, ArmPosition.COLLECT), STOW_TO_CUBE_COLLECT);
        collectCommands.put(new CollectCommandKey(CollectLevel.LOW, ConeOrCube.CUBE, ArmPosition.STOW), CUBE_COLLECT_TO_STOW);
    }

    private ScoringLevel scoreLevel = ScoringLevel.HIGH;
    private ConeOrCube mode = ConeOrCube.CUBE;
    private ArmPosition armPosition = ArmPosition.STOW;
    private Command stowCommand = new InstantCommand();

    public InstantCommand setScoringLevel(ScoringLevel level) {
        InstantCommand cmd = new InstantCommand(() -> {
            scoreLevel = level;
            logger.info("Scoring Level: " + scoreLevel);
        });
        cmd.setName("Scoring Level " + level);
        return cmd;
    }

    public ConeOrCube getGamePieceMode() {
        return mode;
    }
    public InstantCommand toggleGamePieceMode() {
        InstantCommand cmd = new InstantCommand(() -> {
            mode = switch (mode) {
                case CONE -> ConeOrCube.CUBE;

                case CUBE -> ConeOrCube.CONE;
            };
            logger.info("Scoring mode: " + mode);
            if (mode == ConeOrCube.CUBE){
                LED.getInstance().color = LEDConstants.PURPLE;
            } else if (mode == ConeOrCube.CONE) {
                LED.getInstance().color = LEDConstants.YELLOW;
            } else {
                LED.getInstance().color = LEDConstants.BAD;
            }
        });
        cmd.setName("Game Piece Toggle");
        return cmd;
    }

    public InstantCommand gamePieceMode(ConeOrCube mode) {
        var cmd = new InstantCommand(() -> this.mode = mode);
        cmd.setName(mode.name());
        return cmd;
    }

    public Command toggleScoring() {

        return new InstantCommand(() -> {

            switch (armPosition) {

                // Stowed position, so move to score
                case STOW -> {
                    var scoreCommand = new ArmMoveCommand(scoreCommands.get(new ScoreCommandKey(scoreLevel, mode, ArmPosition.SCORE)));
                    scoreCommand.schedule();
                    logger.info("Scoring " + mode + " at " + scoreLevel + " with " + scoreCommand.getName());
                    armPosition = ArmPosition.SCORE;
                    stowCommand = new ArmMoveCommand(scoreCommands.get(new ScoreCommandKey(scoreLevel, mode, ArmPosition.STOW))); // set stow command for next stow
                }

                // Collect position, stow first then move to score
                case COLLECT, SCORE -> {
                    stowCommand.schedule();
                    logger.info("Stowing Arm with " + stowCommand.getName());
                    armPosition = ArmPosition.STOW;
                    stowCommand = new InstantCommand();
                }
            }
        });
    }

    public Command toggleCollecting(CollectLevel collectLevel) {

        return new InstantCommand(() -> {
            switch (armPosition) {

                // Stowed position, so move to score
                case STOW -> {
                    var collectCommand = new ArmMoveCommand(collectCommands.get(new CollectCommandKey(collectLevel, mode, ArmPosition.COLLECT)));
                    collectCommand.schedule();
                    logger.info("Collecting " + mode + " with " + collectCommand.getName());
                    armPosition = ArmPosition.COLLECT;
                    stowCommand = new ArmMoveCommand(collectCommands.get(new CollectCommandKey(collectLevel, mode, ArmPosition.STOW))); // set stow command for next stow
                }

                // Collect position, stow first then move to score
                case SCORE, COLLECT -> {
                    stowCommand.schedule();
                    logger.info("Stowing Arm with " + stowCommand.getName());
                    armPosition = ArmPosition.STOW;
                    stowCommand = new InstantCommand();
                }
            }
        });
    }

    public void reset() {
        armPosition = ArmPosition.STOW;
    }
}
