package org.tahomarobotics.robot.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.tahomarobotics.robot.chassis.Chassis;

public class BalancedCommand extends CommandBase {
    private final Chassis chassis = Chassis.getInstance();

    private static final double GAIN = 1.5;

    private static final double ANGLE = Units.degreesToRadians(5.0);

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        var angle = chassis.getPitch().getRadians();
        if (Math.abs(angle) > ANGLE) {
            chassis.drive(new ChassisSpeeds((DriverStation.getAlliance() == DriverStation.Alliance.Blue ? GAIN : -GAIN) * angle,0,0));
        } else {
            chassis.drive(new ChassisSpeeds());
        }
    }

    @Override
    public void end(boolean interrupted) {
        chassis.drive(new ChassisSpeeds());
    }
}