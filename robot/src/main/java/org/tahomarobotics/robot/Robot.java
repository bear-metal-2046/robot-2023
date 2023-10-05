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
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
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

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.RandomAccessFile;
import java.util.ArrayList;
import java.util.List;



public class Robot extends LoggedRobot {
    private static final Logger logger = LoggerFactory.getLogger(Robot.class);

    private static final File WATCHDOG_FILE = new File("/tmp/robot_monitor.txt");

    @SuppressWarnings("MismatchedQueryAndUpdateOfCollection")
    private final List<SubsystemIF> subsystems = new ArrayList<>();

    private final RandomAccessFile raf;
    private int heartbeat;

    public Robot() {
        RandomAccessFile temp = null;
        try {
            temp = new RandomAccessFile(WATCHDOG_FILE, "rw");
        } catch (FileNotFoundException e) {
            logger.error("Failed to create random access file", e);
        }
        this.raf = temp;
    }
    @Override
    public void robotInit() {

        try {
            SystemLogger.logRobotInit();

            var aklogger = org.littletonrobotics.junction.Logger.getInstance();

            // Record metadata
            aklogger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
            aklogger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
            aklogger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
            aklogger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
            aklogger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
            switch (BuildConstants.DIRTY) {
                case 0:
                    aklogger.recordMetadata("GitDirty", "All changes committed");
                    break;
                case 1:
                    aklogger.recordMetadata("GitDirty", "Uncommitted changes");
                    break;
                default:
                    aklogger.recordMetadata("GitDirty", "Unknown");
                    break;
            }

            if (isReal()) {
                aklogger.addDataReceiver(new WPILOGWriter("/U")); // USB Stick
                aklogger.addDataReceiver(new NT4Publisher()); // Publish to NT
                new PowerDistribution(1, PowerDistribution.ModuleType.kRev); // Power Logging
            } else {
                if (RobotMap.IS_REPLAYING) { // Replay
                    setUseTiming(false); // Run as fast as possible
                    String logPath = LogFileUtil.findReplayLog(); // Find the log or prompt the user.
                    aklogger.setReplaySource(new WPILOGReader(logPath)); // Read the replay log
                    aklogger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save the output to a new log
                } else { // Sim
                    aklogger.addDataReceiver(new WPILOGWriter("")); // Local Folder
                    aklogger.addDataReceiver(new NT4Publisher()); // Publish to NT
                    new PowerDistribution(1, PowerDistribution.ModuleType.kRev); // Power Logging?
                }
            }

            aklogger.start();

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
        } catch(Throwable t) {
            shutdown(t);
        }
    }

    @Override
    public void robotPeriodic() {

        updateWatchdog();
        try {
            CommandScheduler.getInstance().run();
        } catch (Throwable t) {
            shutdown(t);
        }
    }


    private void updateWatchdog() {

        try {
            if (raf != null) {
                raf.seek(0);
                raf.writeBytes(Integer.toString(heartbeat++) + "\n");
            }
        } catch (IOException e) {
            logger.error("failed to write to watchdog file", e);
        }
    }

    @Override
    public void autonomousInit() {
        try {
            SystemLogger.logAutoInit();

            subsystems.forEach(SubsystemIF::onAutonomousInit);

            logger.info("-=-=-=- AUTONOMOUS initiated -=-=-=-");

        } catch (Throwable t) {
            shutdown(t);
        }
    }

    @Override
    public void teleopInit() {
        try {
            SystemLogger.logTeleopInit();

            subsystems.forEach(SubsystemIF::onTeleopInit);

            logger.info("-=-=-=- TELEOP initiated -=-=-=-");
        } catch (Throwable t) {
            shutdown(t);
        }
    }

    @Override
    public void disabledInit() {
        try {
            SystemLogger.logDisabledInit();

            subsystems.forEach(SubsystemIF::onDisabledInit);

            logger.info("-=-=-=- DISABLE initiated -=-=-=-");
        } catch (Throwable t) {
            shutdown(t);
        }
    }
    @Override
    public void testInit() {
        try {
            SystemLogger.logTestInit();

            logger.info("-=-=-=- TEST initiated -=-=-=-");

        } catch (Throwable t) {
            shutdown(t);
        }
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

    private void shutdown(Throwable t) {
        t.printStackTrace(System.err);
        SystemLogger.logThrowableCrash(t);
        Runtime.getRuntime().halt(1);
    }
}
