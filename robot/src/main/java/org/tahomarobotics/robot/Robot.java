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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.arm.Arm;
import org.tahomarobotics.robot.auto.Autonomous;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.climb.Beacher;
import org.tahomarobotics.robot.climb.Paw;
import org.tahomarobotics.robot.grabber.Grabber;
import org.tahomarobotics.robot.lights.LED;
import org.tahomarobotics.robot.lights.LEDConstants;
import org.tahomarobotics.robot.util.SparkMaxHelper;
import org.tahomarobotics.robot.util.SystemLogger;
import org.tahomarobotics.robot.wrist.Wrist;

import java.util.ArrayList;
import java.util.List;



public class Robot extends TimedRobot {
    private static final Logger logger = LoggerFactory.getLogger(Robot.class);

    @SuppressWarnings("MismatchedQueryAndUpdateOfCollection")
    private final List<SubsystemIF> subsystems = new ArrayList<>();

    @Override
    public void robotInit() {

        SystemLogger.logRobotInit();

        DriverStation.silenceJoystickConnectionWarning(true);

        subsystems.add(Chassis.getInstance().initialize());
        subsystems.add(Arm.getInstance().initialize());
        subsystems.add(Grabber.getInstance().initialize());
        subsystems.add(Wrist.getInstance().initialize());
        subsystems.add(Paw.getLeftInstance().initialize());
        subsystems.add(Paw.getRightInstance().initialize());
        subsystems.add(Beacher.getBeacherInstance().initialize());
        subsystems.add(Autonomous.getInstance().initialize());
        subsystems.add(OI.getInstance().initialize());

        SparkMaxHelper.clear();

        LED.getInstance().setLEDColor(LEDConstants.PARTY_MODE);

        logger.info("Robot Initialized.");
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {

        SystemLogger.logAutoInit();

        subsystems.forEach(SubsystemIF::onAutonomousInit);

        logger.info("-=-=-=- AUTONOMOUS initiated -=-=-=-");
    }

    @Override
    public void teleopInit() {
        SystemLogger.logTeleopInit();

        subsystems.forEach(SubsystemIF::onTeleopInit);

        logger.info("-=-=-=- TELEOP initiated -=-=-=-");
    }

    @Override
    public void disabledInit() {
        SystemLogger.logDisabledInit();

        subsystems.forEach(SubsystemIF::onDisabledInit);

        logger.info("-=-=-=- DISABLE initiated -=-=-=-");
    }
    @Override
    public void testInit() {
        SystemLogger.logTestInit();

        logger.info("-=-=-=- TEST initiated -=-=-=-");
    }

    @Override
    public void autonomousPeriodic() {}
    @Override
    public void simulationPeriodic() {}
    @Override
    public void teleopPeriodic() {}
    @Override
    public void testPeriodic() {}
    @Override
    public void disabledPeriodic() {}
}
