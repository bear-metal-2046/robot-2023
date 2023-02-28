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
    private static final Command NULL_ARM_MOVE = new InstantCommand();
    private record ScoreCommandKey(ScoringLevel level, ConeOrCube mode, ArmPosition armPosition) {}
    private record CollectCommandKey(CollectLevel level, ArmPosition armPosition) {}
    private final Map<ScoreCommandKey, Command> scoreCommands = new HashMap<>();
    private final Map<CollectCommandKey, Command> collectCommands = new HashMap<>();

    public OperatorArmMoveSelection() {
        scoreCommands.put(new ScoreCommandKey(ScoringLevel.HIGH, ConeOrCube.CUBE, ArmPosition.SCORE), STOW_TO_HIGH_BOX);
        scoreCommands.put(new ScoreCommandKey(ScoringLevel.HIGH, ConeOrCube.CUBE, ArmPosition.STOW), HIGH_BOX_TO_STOW);
        scoreCommands.put(new ScoreCommandKey(ScoringLevel.HIGH, ConeOrCube.CONE, ArmPosition.SCORE), STOW_TO_HIGH_POLE);
        scoreCommands.put(new ScoreCommandKey(ScoringLevel.HIGH, ConeOrCube.CONE, ArmPosition.STOW), HIGH_POLE_TO_STOW);

        scoreCommands.put(new ScoreCommandKey(ScoringLevel.MID, ConeOrCube.CUBE, ArmPosition.SCORE), STOW_TO_MID_BOX);
        scoreCommands.put(new ScoreCommandKey(ScoringLevel.MID, ConeOrCube.CUBE, ArmPosition.STOW), MID_BOX_TO_STOW);
        scoreCommands.put(new ScoreCommandKey(ScoringLevel.MID, ConeOrCube.CONE, ArmPosition.SCORE), STOW_TO_MID_POLE);
        scoreCommands.put(new ScoreCommandKey(ScoringLevel.MID, ConeOrCube.CONE, ArmPosition.STOW), MID_POLE_TO_STOW);

        collectCommands.put(new CollectCommandKey(CollectLevel.FEEDER, ArmPosition.COLLECT), STOW_TO_FEEDER_COLLECT);
        collectCommands.put(new CollectCommandKey(CollectLevel.FEEDER, ArmPosition.STOW), FEEDER_COLLECT_TO_STOW);
        collectCommands.put(new CollectCommandKey(CollectLevel.LOW, ArmPosition.COLLECT), STOW_TO_DOWN_COLLECT);
        collectCommands.put(new CollectCommandKey(CollectLevel.LOW, ArmPosition.STOW), DOWN_COLLECT_TO_STOW);
    }

    private ScoringLevel scoreLevel = ScoringLevel.HIGH;
    private ConeOrCube mode = ConeOrCube.CUBE;
    private ArmPosition armPosition = ArmPosition.STOW;
    private Command stowCommand = NULL_ARM_MOVE;

    public Command setScoringLevel(ScoringLevel level) {
        return new InstantCommand(() -> {
            scoreLevel = level;
            logger.info("Scoring Level: " + scoreLevel);
        });
    }

    public Command toggleGamePieceMode() {
        return new InstantCommand(() -> {
            mode = switch (mode) {
                case CONE -> ConeOrCube.CUBE;
                case CUBE -> ConeOrCube.CONE;
            };
            logger.info("Scoring mode: " + mode);
        });
    }

    public Command toggleScoring() {

        return new InstantCommand(() -> {

            switch (armPosition) {

                // Stowed position, so move to score
                case STOW -> {
                    scoreCommands.get(new ScoreCommandKey(scoreLevel, mode, ArmPosition.SCORE)).schedule();
                    armPosition = ArmPosition.SCORE;
                    stowCommand = scoreCommands.get(new ScoreCommandKey(scoreLevel, mode, ArmPosition.STOW)); // set stow command for next stow
                    logger.info("Scoring at " + scoreLevel + " with " + mode);
                }

                // Collect position, stow first then move to score
                case COLLECT, SCORE -> {
                    stowCommand.schedule();
                    armPosition = ArmPosition.STOW;
                    stowCommand = NULL_ARM_MOVE;
                    logger.info("Scoring Stowing");
                }
            }
        });
    }

    public Command toggleCollecting(CollectLevel collectLevel) {

        return new InstantCommand(()-> {
            switch (armPosition) {

                // Stowed position, so move to score
                case STOW -> {
                    collectCommands.get(new CollectCommandKey(collectLevel, ArmPosition.COLLECT)).schedule();
                    armPosition = ArmPosition.COLLECT;
                    stowCommand = collectCommands.get(new CollectCommandKey(collectLevel, ArmPosition.STOW)); // set stow command for next stow
                    logger.info("Collecting at level: " + collectLevel);
                }

                // Collect position, stow first then move to score
                case SCORE, COLLECT -> {
                    stowCommand.schedule();
                    armPosition = ArmPosition.STOW;
                    stowCommand = NULL_ARM_MOVE;
                    logger.info("Collection Stowing");
                }
            }
        });
    }
}
