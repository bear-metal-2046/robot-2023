package org.tahomarobotics.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.chassis.Chassis;

import java.util.ArrayList;

public class Robot extends TimedRobot
{
    private Logger logger = LoggerFactory.getLogger(Robot.class);

    private ArrayList<Subsystem> instances = new ArrayList<>();

    @Override
    public void robotInit()
    {
        logger.info("Hit Initialization.");
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
        logger.info("Hit Autonomous Initialization.");
        DriverStation.reportWarning("-=-=-=- AUTONOMOUS ENABLED -=-=-=-", false);
    }

    @Override
    public void autonomousPeriodic() {}
    
    
    @Override
    public void teleopInit()
    {
        logger.info("Hit Teleoperated Initialization.");
        DriverStation.reportWarning("-=-=-=- TELEOP ENABLED -=-=-=-", false);
        DriverStation.reportWarning("FRC 2023 COMPETITION ROBOT 'AEGIS'", false);
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
