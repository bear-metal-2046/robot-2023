package org.tahomarobotics.robot.chassis.module;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Base Interface for SwerveModules
 * Helps with abstraction of modules to chassis, primarily to permit multiple chassis types.
 */
public interface SwerveModuleIF {

    /**
     * Fetches the current position of the swerve module.
     * @return SwerveModulePosition, the current position of the module.
     */
    SwerveModulePosition getPosition();

    /**
     * Allows you to set the state of the SwerveModule.
     * @param desiredState the desired state for the module.
     */
    void setDesiredState(SwerveModuleState desiredState);

    /**
     * Set voltage for the module
     * @param voltage the voltage to be set
     */
    void setDriveVoltage(double voltage);

    /**
     * Alignment method for the module
     */
    void align();

    /**
     * Zeroes the current offset of the module.
     */
    void zeroOffset();

    /**
     * Update the offset of the module.
     */
    void updateOffset();

    /**
     * Displays the position of the module to dashboard.
     */
    void displayPosition();

    /**
     * Gets the current velocity of the module.
     * @return double, velocity typically in radians.
     */
    double getVelocity();

    /**
     * Current state of the module.
     * @return SwerveModuleState, the current state.
     */
    SwerveModuleState getState();

    /**
     * Gets the absolute angle of the swerve module.
     * @return double, the angle usually in radians.
     */
    double getAbsoluteAngle();
}
