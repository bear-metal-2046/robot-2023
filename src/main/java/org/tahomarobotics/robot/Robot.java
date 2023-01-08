package org.tahomarobotics.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.util.LoggerManager;

import java.util.ArrayList;

public class Robot extends TimedRobot
{
    private ArrayList<Subsystem> instances = new ArrayList<>();

    @Override
    public void robotInit()
    {
        LoggerManager.log("Hit Initialization.");
        instances.add(Chassis.getInstance());
    }

    @Override
    public void robotPeriodic()
    {
        CommandScheduler.getInstance().run();
    }
    
    
    @Override
    public void disabledInit() {}
    
    
    @Override
    public void disabledPeriodic() {}
    
    @Override
    public void autonomousInit()
    {
        LoggerManager.log("Hit Autonomous Initialization.");
        LoggerManager.warn("-=-=-=- AUTONOMOUS ENABLED -=-=-=-");
    }

    @Override
    public void autonomousPeriodic() {}
    
    
    @Override
    public void teleopInit()
    {
        LoggerManager.log("Hit Teleoperated Initialization.");
        LoggerManager.warn("-=-=-=- TELEOP ENABLED -=-=-=-");
        LoggerManager.warn("FRC 2023 COMPETITION ROBOT 'AEGIS'");
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
