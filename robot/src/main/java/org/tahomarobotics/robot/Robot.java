package org.tahomarobotics.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.tahomarobotics.robot.OI.OI;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.configuration.RobotConfig;
import org.tahomarobotics.robot.util.LoggerManager;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.InputStream;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.Scanner;

public class Robot extends TimedRobot {

    private static Robot INSTANCE;

    //Configuration Path; Currently held at project dir.
    private static final String CONFIG_PATH = "config.yml";

    static {
        // Initialize use of async loggers. Only to be done statically in the main class.
        System.setProperty("log4j2.contextSelector",
                "org.apache.logging.log4j.core.async.AsyncLoggerContextSelector");
    }

    @SuppressWarnings(value = "MismatchedQueryAndUpdateOfCollection")
    private final ArrayList<Object> instances = new ArrayList<>();

    private RobotConfig config;

    /**
     * Ran on Code startup by RoboRIO Java Runtime
     */
    @Override
    public void robotInit()
    {
        //Do NOT modify
        //If you need an explanation: this ensures that the static instance of this class is ALWAYS what the RoboRIO is currently running.
        INSTANCE = this;
//        RobotConfig robotConfig = new RobotConfig(tryLoadConfig());
//        this.config = robotConfig;
//        LoggerManager.log("Configuration Loaded.");
        //You can modify now

        instances.add(Chassis.getInstance().initialize());
        instances.add(OI.getInstance());

        LoggerManager.log("Robot Initialized.");
        LoggerManager.log("Now with AprilTags!");
    }

    /**
     * Attempts to load a pre-existing config, if one is not present it will generate the default one.
     * @return Newly created or pre-existing configuration file.
     */
    public File tryLoadConfig() {
        File configFile = new File(CONFIG_PATH);
        LoggerManager.log("Config Path: " + configFile.toPath());
        if(!configFile.exists()) {
            LoggerManager.warn("Configuration File does not exist, attempting creation of default config.");
            try (InputStream is = this.getClass().getClassLoader().getResourceAsStream("config.yml")) {
                Files.copy(is, configFile.toPath());
                LoggerManager.warn("Default Configuration saved to run directory. You WILL need to modify this for the robot to function.");
            } catch (Exception ex) {
                LoggerManager.error("Failed to save configuration, check file permission?");
                ex.printStackTrace();
            }
        }
        return configFile;
    }

    @Override
    public void robotPeriodic()
    {
        CommandScheduler.getInstance().run();
    }
    
    
    @Override
    public void disabledInit() {
        Chassis.getInstance().closeVision();
    }
    
    
    @Override
    public void disabledPeriodic() {}
    
    @Override
    public void autonomousInit()
    {
        LoggerManager.warn("-=-=-=- AUTONOMOUS ENABLED -=-=-=-");
    }

    @Override
    public void autonomousPeriodic() {}
    
    
    @Override
    public void teleopInit()
    {
        LoggerManager.warn("-=-=-=- TELEOP ENABLED -=-=-=-");
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

    public static void main(String... args)
    {
        RobotBase.startRobot(Robot::new);
    }

    public static RobotConfig getConfig() {
        return Robot.INSTANCE.config;
    }
}
