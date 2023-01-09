package org.tahomarobotics.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.tahomarobotics.robot.OI.OI;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.util.LoggerManager;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Scanner;

public class Robot extends TimedRobot
{
    static {
        // Initialize use of async loggers. Only to be done statically in the main class.
        System.setProperty("log4j2.contextSelector",
                "org.apache.logging.log4j.core.async.AsyncLoggerContextSelector");
    }

    @SuppressWarnings(value = "MismatchedQueryAndUpdateOfCollection")
    private final ArrayList<Object> instances = new ArrayList<>();

    @Override
    public void robotInit()
    {
        logVersion();

        instances.add(Chassis.getInstance().initialize());
        instances.add(OI.getInstance());

        LoggerManager.log("Robot Initialized.");
    }

    public void logVersion() {
        File deployDir = Filesystem.getDeployDirectory();
        File branchFile = new File(deployDir, "branch.txt");
        File commitFile = new File(deployDir, "commit.txt");

        String branch;
        try {
            Scanner s = new Scanner(branchFile);
            branch = s.next();
            s.close();
        } catch (FileNotFoundException e) {
            branch = "<Not Found>";
        }

        String commit;
        try {
            Scanner s = new Scanner(commitFile);
            commit = s.next();
            s.close();
        } catch (FileNotFoundException e) {
            commit = "<Not Found>";
        }

        LoggerManager.warn(String.format("Current Version: %s | %s", branch, commit));
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
}
