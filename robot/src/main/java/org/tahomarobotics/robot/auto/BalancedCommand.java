package org.tahomarobotics.robot.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.tahomarobotics.robot.chassis.Chassis;

public class BalancedCommand extends CommandBase {
    private final Chassis chassis = Chassis.getInstance();
    @Override
    public void execute() {
        if(chassis.getPitch().getDegrees() > 2.5){ chassis.drive(new ChassisSpeeds(-.4,0,0));}
        if(chassis.getPitch().getDegrees() < -2.5){ chassis.drive(new ChassisSpeeds(.4,0,0));}
    }

    @Override
    public boolean isFinished() {
        return Math.abs(chassis.getPitch().getDegrees()) < 2.5;
    }

    @Override
    public void end(boolean interrupted) {
        chassis.drive(new ChassisSpeeds());
    }
}
