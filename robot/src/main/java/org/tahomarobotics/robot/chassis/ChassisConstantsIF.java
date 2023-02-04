package org.tahomarobotics.robot.chassis;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

import java.io.Serializable;
import java.util.List;

/**
 * SwerveConstants Interface
 * This acts as a basic interface for Chassis in order to allow for various types of chassis.
 */
public interface ChassisConstantsIF {

    /**
     * Return the set of swerve modules
     * @return List<SwerveModuleIF>, list of modules
     */
    List<SwerveModuleIF> createSwerveModules(Double angularOffsets[]);

    /**
     * The calculated maximum speed of the robot
     * @return double, in meters per second.
     */
    double maxAttainableMps();

    /**
     * Fetches the current acceleration limit.
     * @return double, the limit in meters per second.
     */
    double accelerationLimit();

    /**
     * Fetches the current angular (rotation) acceleration limit.
     * @return double, the limit in meters per second.
     */
    double angularAccelerationLimit();

    /**
     * Fetches currently active SwerveDriveKinematics.
     * @return SwerveDriveKinematics
     */

    double  maxRotationalVelocity();

}
