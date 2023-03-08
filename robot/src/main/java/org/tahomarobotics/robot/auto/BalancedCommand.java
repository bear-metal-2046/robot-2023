package org.tahomarobotics.robot.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.tahomarobotics.robot.chassis.Chassis;

public class BalancedCommand extends CommandBase {
    private final Timer timer = new Timer();
    private final Chassis chassis = Chassis.getInstance();

    private static final double GAIN = 1.83;

    private static final double ANGLE = Units.degreesToRadians(5.0);

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        var angle = chassis.getPitch().getRadians();
        if (Math.abs(angle) > ANGLE) {
            timer.reset();
            chassis.drive(new ChassisSpeeds(-GAIN * angle,0,0));
        }
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(0.1);
    }

    @Override
    public void end(boolean interrupted) {
        chassis.drive(new ChassisSpeeds());
    }
}