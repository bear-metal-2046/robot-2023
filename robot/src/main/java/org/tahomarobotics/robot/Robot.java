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

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.chassis.Chassis;

import java.util.ArrayList;

public class Robot extends TimedRobot {

    private static final Logger logger = LoggerFactory.getLogger(Robot.class);


    static {
        // Initialize use of async loggers. Only to be done statically in the main class.
        System.setProperty("log4j2.contextSelector",
                "org.apache.logging.log4j.core.async.AsyncLoggerContextSelector");
    }

    @SuppressWarnings(value = "MismatchedQueryAndUpdateOfCollection")
    private final ArrayList<Object> instances = new ArrayList<>();

    /**
     * Ran on Code startup by RoboRIO Java Runtime
     */
    @Override
    public void robotInit() {
        instances.add(Chassis.getInstance().initialize());
        instances.add(OI.getInstance());

        logger.info("Robot Initialized.");
    }

    @Override
    public void robotPeriodic()
    {
        CommandScheduler.getInstance().run();
    }
    
    @Override
    public void disabledInit() {
    }
    
    
    @Override
    public void disabledPeriodic() {}
    
    @Override
    public void autonomousInit()
    {
        logger.info("-=-=-=- AUTONOMOUS ENABLED -=-=-=-");
    }

    @Override
    public void autonomousPeriodic() {}
    
    
    @Override
    public void teleopInit()
    {
        logger.warn("-=-=-=- TELEOP ENABLED -=-=-=-");
    }
    
    
    @Override
    public void teleopPeriodic() {
        OI.getInstance().teleopPeriodic();
    }
    
    
    @Override
    public void testInit()
    {
        CommandScheduler.getInstance().cancelAll();
    }
    
    @Override
    public void testPeriodic() {}

    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}
