package org.tahomarobotics.robot.OI;

import edu.wpi.first.wpilibj.XboxController;
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
        AButton.whenPressed(Chassis.getInstance()::orientToZeroHeading);
        JoystickButton BButton = new JoystickButton(driveController, 2);
        BButton.whenPressed(Chassis.getInstance()::toggleOrientation);
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
