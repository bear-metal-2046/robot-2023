package org.tahomarobotics.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.tahomarobotics.robot.OI.OI;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.util.LoggerManager;

import java.util.ArrayList;

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
        instances.add(Chassis.getInstance().initialize());
        instances.add(OI.getInstance());

        LoggerManager.log("Robot Initialized.");
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
    public void teleopPeriodic() {}
    
    
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
