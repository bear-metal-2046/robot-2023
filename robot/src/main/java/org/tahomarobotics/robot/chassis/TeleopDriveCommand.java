package org.tahomarobotics.robot.chassis;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

/**
 * Basic Teleoperated Drive Command
 * Used to make the robot go vrmmm
 */

public class TeleopDriveCommand extends CommandBase {
    private final DoubleSupplier xSup, ySup, rotSup;

    private double xyFactor = 1, rotFactor = 1;

    private final Chassis chassis = Chassis.getInstance();

    public TeleopDriveCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rotation) {
        this.xSup = x;
        this.ySup = y;
        this.rotSup = rotation;

        addRequirements(chassis);
    }

    @Override
    public void execute() {
        chassis.drive(
            xSup.getAsDouble() * xyFactor,
            ySup.getAsDouble() * xyFactor,
            rotSup.getAsDouble() * rotFactor
        );
    }

    @Override
    public void end(boolean interrupted) {
        chassis.drive(0.0, 0.0, 0.0);
    }

    public void reduceSpeed(double xyFactor, double rotFactor) {
        this.xyFactor = xyFactor;
        this.rotFactor = rotFactor;
    }
}
