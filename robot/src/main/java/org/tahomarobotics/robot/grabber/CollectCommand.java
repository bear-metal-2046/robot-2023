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
package org.tahomarobotics.robot.grabber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import static org.tahomarobotics.robot.grabber.GrabberConstants.*;

public class CollectCommand extends CommandBase {

    private enum MovementState {  OFF, INJEST, RETAIN, EJECT, SCORE  }

    private final DoubleSupplier driveLeftTriger;
    private final DoubleSupplier manipLeftTriger;
    private final IntSupplier povDir;
    private final Timer timer = new Timer();
    private MovementState state = MovementState.OFF;
    Grabber grabber = Grabber.getInstance();
    public CollectCommand(DoubleSupplier driveLeftTriger, DoubleSupplier manipLeftTriger, IntSupplier povDir) {
        this.driveLeftTriger = driveLeftTriger;
        this.manipLeftTriger = manipLeftTriger;
        this.povDir = povDir;
        addRequirements(grabber);
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        MovementState oldState = state;

        double leftDriverTrigger = driveLeftTriger.getAsDouble();
        double injestSpeed = Math.max(leftDriverTrigger * MAX_SPEED, RETAIN_SPEED);
        double leftManipTrigger = manipLeftTriger.getAsDouble();

        switch (state) {
            case OFF -> {
                grabber.setGrabberSpeed(0);

                if (leftDriverTrigger > TRIGGER_DEAD_ZONE) {
                    state = MovementState.INJEST;
                    timer.reset();

                } else if (leftManipTrigger > TRIGGER_DEAD_ZONE) {
                    state = MovementState.SCORE;
                }
            }
            case INJEST -> {

                if (isNotStalled()) {
                    timer.reset();
                }

                // stalled for too long
                if (timer.get() > GrabberConstants.INTAKE_TIMOUT ) {
                    injestSpeed = RETAIN_SPEED;
                }

                if (leftDriverTrigger < TRIGGER_DEAD_ZONE) {
                    state = MovementState.RETAIN;
                }


                if (povDir.getAsInt() == 180) {
                    state = MovementState.EJECT;
                }

                grabber.setGrabberSpeed(injestSpeed);
            }
            case RETAIN -> {

                grabber.setGrabberSpeed(RETAIN_SPEED);

                if (leftDriverTrigger > TRIGGER_DEAD_ZONE) {
                    state = MovementState.INJEST;
                }

                if (povDir.getAsInt() == 180) {
                    state = MovementState.EJECT;
                }

                if (leftManipTrigger > TRIGGER_DEAD_ZONE) {
                    state = MovementState.SCORE;
                }

            }
            case EJECT -> {
                grabber.setGrabberSpeed(GrabberConstants.EJECT_SPEED);

                if (povDir.getAsInt() != 180) {
                    state = MovementState.OFF;
                }
            }
            case SCORE -> {
                grabber.setGrabberSpeed(-leftManipTrigger * MAX_SPEED);
                if (leftManipTrigger < TRIGGER_DEAD_ZONE) {
                    state = MovementState.OFF;
                }
            }
        }
        SmartDashboard.putString("State", state.toString());
    }

    private boolean isNotStalled() {
        return Math.abs(grabber.getVelocity()) > 10;
    }



}
