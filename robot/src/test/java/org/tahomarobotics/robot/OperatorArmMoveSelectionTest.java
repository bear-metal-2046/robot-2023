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

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.Random;
import java.util.concurrent.Callable;

import static org.junit.jupiter.api.Assertions.fail;

public class OperatorArmMoveSelectionTest {
    private static final OperatorArmMoveSelection selector = new OperatorArmMoveSelection();
    private enum Button {
        DriverLB(() -> selector.toggleCollecting(OperatorArmMoveSelection.CollectLevel.FEEDER)),
        DriverRB(() -> selector.toggleCollecting(OperatorArmMoveSelection.CollectLevel.LOW)),
        ManipRB(() -> selector.toggleScoring()),
        ManipNorth(() -> selector.setScoringLevel(OperatorArmMoveSelection.ScoringLevel.HIGH)),
        ManipEast(() -> selector.setScoringLevel(OperatorArmMoveSelection.ScoringLevel.MID)),
        //ManipSouth(() -> selector.setScoringLevel(OperatorArmMoveSelection.ScoringLevel.LOW)),
        X(() -> selector.toggleGamePieceMode());

        private final Callable<Command> func;

        Button(Callable<Command> func) {
            this.func = func;
        }

        public Command process() throws Exception {
            return func.call();
        }
    }

    private final Button buttons[] = {
            Button.ManipNorth,
            Button.X,
            Button.DriverRB,
            Button.DriverRB,
            Button.ManipNorth,
            Button.X,
            Button.ManipNorth,
            Button.ManipEast,
            Button.ManipRB,
            Button.X,
            Button.DriverRB,
            Button.DriverRB,
            Button.ManipEast,
            Button.ManipEast,
            Button.ManipEast,
            Button.ManipRB,
            Button.X,
            Button.X,
            Button.X,
            Button.ManipNorth,
            Button.ManipNorth,
            Button.ManipRB,
            Button.DriverLB,
            Button.DriverRB,
            Button.DriverRB,
            Button.DriverLB,
            Button.DriverLB,
            Button.ManipNorth,
            Button.DriverLB,
            Button.X,
            Button.X,
            Button.DriverRB,
            Button.ManipRB,
            Button.ManipEast,
            Button.ManipRB,
            Button.ManipRB,
            Button.ManipRB,
            Button.ManipEast,
            Button.ManipEast,
            Button.DriverLB,
            Button.ManipEast,
            Button.ManipEast,
            Button.DriverLB,
            Button.DriverRB,
            Button.X,
            Button.ManipNorth,
            Button.DriverLB,
            Button.ManipRB,
            Button.DriverRB,
            Button.DriverLB,
            Button.DriverLB,
            Button.ManipEast,
            Button.DriverRB,
            Button.DriverLB,
            Button.ManipEast,
            Button.ManipNorth,
            Button.DriverRB,
            Button.DriverLB,
            Button.X,
            Button.ManipEast,
            Button.ManipRB,
            Button.DriverLB,
            Button.X,
            Button.X,
            Button.DriverLB,
            Button.ManipEast,
            Button.DriverLB,
            Button.ManipNorth,
            Button.DriverRB,
            Button.DriverRB,
            Button.ManipRB,
            Button.DriverLB,
            Button.DriverRB,
            Button.ManipNorth,
            Button.DriverRB,
            Button.ManipNorth,
            Button.ManipRB,
            Button.DriverRB,
            Button.ManipRB,
            Button.ManipRB,
            Button.X,
            Button.DriverLB,
            Button.ManipRB,
            Button.ManipEast,
            Button.ManipNorth,
            Button.ManipEast,
            Button.ManipRB,
            Button.ManipEast,
            Button.DriverRB,
            Button.ManipEast,
            Button.DriverLB,
            Button.DriverLB,
            Button.ManipEast,
            Button.ManipEast,
            Button.ManipNorth,
            Button.DriverRB,
            Button.ManipEast,
            Button.DriverLB,
            Button.ManipNorth,
            Button.ManipRB,
    };

    @BeforeEach
        // this method will run before each test
    void setup() {
        assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
        DriverStationSim.setEnabled(true);
    }

    @SuppressWarnings("PMD.SignatureDeclareThrowsException")
    @AfterEach
        // this method will run after each test
    void shutdown() throws Exception {
        DriverStationSim.setEnabled(false);
    }


    @Test
    void testAllCombination() {

        DriverStationSim.setEnabled(true);

        for (Button button : buttons) {

            try {
                Command cmd = button.process();
                cmd.ignoringDisable(true).schedule();
                while(cmd.isScheduled()) {
                    Thread.currentThread().wait(20);
                }


            } catch (Exception e) {
                fail("OperatorArmMoveSelection threw exception: ", e);
            }
        }

    }
}
