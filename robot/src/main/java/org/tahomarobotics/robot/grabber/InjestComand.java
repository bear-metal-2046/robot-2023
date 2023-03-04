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
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class InjestComand extends CommandBase {

    private static final Logger logger = LoggerFactory.getLogger(InjestComand.class);
    private static final double INJEST_DURATION = 1.0;
    private static final double INJEST_LEVEL = 1.0;
    private final Grabber grabber = Grabber.getInstance();

    private final Timer timer = new Timer();

    public InjestComand() {
        addRequirements(grabber);
    }

    @Override
    public void initialize() {
        timer.restart();
        grabber.ingest(INJEST_LEVEL);
    }

    @Override
    public void end(boolean interrupted) {
        grabber.retain();
        logger.info(getName() + " completed");
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(INJEST_DURATION);
    }
}