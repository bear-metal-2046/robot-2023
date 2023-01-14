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
package org.tahomarobotics.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.chassis.ChassisConstants;
import org.tahomarobotics.robot.chassis.TeleopDriveCommand;

public final class OI
{
    private static final OI INSTANCE = new OI();
    public static OI getInstance() {return INSTANCE;}

    private static final double ROTATIONAL_SENSITIVITY = 3;
    private static final double FORWARD_SENSITIVITY = 3;

    private final XboxController driveController = new XboxController(0);
    private final XboxController manipController = new XboxController(1);

    private OI() {
        bindButtons();

        Chassis.getInstance().setDefaultCommand(
                new TeleopDriveCommand(
                        () -> -desensitizePowerBased(driveController.getLeftY(), FORWARD_SENSITIVITY)
                                * ChassisConstants.MAX_VELOCITY_MPS,
                        () -> -desensitizePowerBased(driveController.getLeftX(), FORWARD_SENSITIVITY)
                                * ChassisConstants.MAX_VELOCITY_MPS,
                        () -> -desensitizePowerBased(driveController.getRightX(), ROTATIONAL_SENSITIVITY)
                                * ChassisConstants.MAX_ANGULAR_VELOCITY_RPS)
        );

        JoystickButton AButton = new JoystickButton(driveController, 1);
        AButton.onTrue(new InstantCommand(Chassis.getInstance()::orientToZeroHeading));

        JoystickButton BButton = new JoystickButton(driveController, 2);
        BButton.onTrue(new InstantCommand(Chassis.getInstance()::toggleOrientation));
    }

    private void bindButtons()
    {

    }

    public void teleopPeriodic() {

    }

    public double getDriveLeftYJoystick() {
        return -desensitizePowerBased(driveController.getLeftY(), FORWARD_SENSITIVITY);
    }

    public double getDriveRightXJoystick() {
        return -desensitizePowerBased(driveController.getRightX(), ROTATIONAL_SENSITIVITY);
    }


    private static final double DEAD_ZONE = 15.0 / 127.0;

    private static double deadband(double value) {
        if (Math.abs(value) > OI.DEAD_ZONE) {
            if (value > 0.0) {
                return (value - OI.DEAD_ZONE) / (1.0 - OI.DEAD_ZONE);
            } else {
                return (value + OI.DEAD_ZONE) / (1.0 - OI.DEAD_ZONE);
            }
        } else {
            return 0.0;
        }
    }

    /**
     * Reduces the sensitivity around the zero point to make the Robot more
     * controllable.
     *
     * @param value - raw input
     * @param power - 1.0 indicates linear (full sensitivity) - larger number
     *              reduces small values
     *
     * @return 0 to +/- 100%
     */
    private double desensitizePowerBased(double value, double power) {
        value = deadband(value);
        return Math.pow(Math.abs(value), power - 1) * value;
    }
}
