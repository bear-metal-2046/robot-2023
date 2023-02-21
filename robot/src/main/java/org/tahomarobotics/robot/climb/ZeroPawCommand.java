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
package org.tahomarobotics.robot.climb;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class ZeroPawCommand extends CommandBase {
    private static final Logger logger = LoggerFactory.getLogger(ZeroPawCommand.class);
    private static final double TIMEOUT = 8;

    private static final double STOPPED_VELOCITY_THREASHOLD = 0.001;
    private static final double INITIAL_MOVE_TIME = 0.1;
    private final Paw paw;
    private final Timer timer = new Timer();

    public ZeroPawCommand(Paw paw) {
        this.paw = paw;
    }
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        paw.setZeroCurrentLimit(true);
        paw.setPower(-0.05);
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("Velocity", paw.getVelocity());
    }

    private boolean hasStopped() {
        return Math.abs(paw.getVelocity()) < STOPPED_VELOCITY_THREASHOLD;
    }

    @Override
    public boolean isFinished() {
        return ( timer.hasElapsed(INITIAL_MOVE_TIME) && hasStopped() ) || timer.hasElapsed(TIMEOUT );
    }

    @Override
    public void end(boolean interrupted) {

        if (timer.hasElapsed(TIMEOUT)) {
            logger.warn(paw.getName() + " zero command has timedout");
        }

        paw.setPower(0);
        paw.zeroEncoder();
        paw.setZeroCurrentLimit(false);

        logger.info(paw.getName() + " zeroing complete");
    }
}

