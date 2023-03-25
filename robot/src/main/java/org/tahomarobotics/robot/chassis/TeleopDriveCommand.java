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
package org.tahomarobotics.robot.chassis;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.tahomarobotics.robot.util.SwerveRateLimiter;

import java.util.function.DoubleSupplier;

public class TeleopDriveCommand extends CommandBase {

    private static final Chassis chassis = Chassis.getInstance();

    private final ChassisSpeeds velocityInput = new ChassisSpeeds();

    private SwerveRateLimiter rateLimiter;

    private final DoubleSupplier xSup, ySup, rotSup;

    private final double maxVelocity;
    private final double maxRotationalVelocity;

    public TeleopDriveCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rotation) {
        this.xSup = x;
        this.ySup = y;
        this.rotSup = rotation;

        addRequirements(chassis);

        ChassisConstantsIF constants = chassis.getSwerveConstants();

        rateLimiter = new SwerveRateLimiter(
                constants.accelerationLimit(),
                constants.angularAccelerationLimit());

        maxVelocity = constants.maxAttainableMps();
        maxRotationalVelocity = constants.maxRotationalVelocity();
    }

    @Override
    public void execute() {

        double direction = DriverStation.getAlliance() == DriverStation.Alliance.Blue ? 1.0 : -1.0;

        velocityInput.vxMetersPerSecond = xSup.getAsDouble() * maxVelocity * direction;
        velocityInput.vyMetersPerSecond = ySup.getAsDouble() * maxVelocity * direction;
        velocityInput.omegaRadiansPerSecond = rotSup.getAsDouble() * maxRotationalVelocity;

        ChassisSpeeds velocityOutput = rateLimiter.calculate(velocityInput);

        chassis.drive(velocityOutput);
    }

    @Override
    public void end(boolean interrupted) {
        chassis.drive(new ChassisSpeeds());
    }

}
