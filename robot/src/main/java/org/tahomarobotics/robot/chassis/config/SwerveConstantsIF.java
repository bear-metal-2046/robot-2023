package org.tahomarobotics.robot.chassis.config;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * SwerveConstants Interface
 * This acts as a basic interface for Chassis in order to allow for various types of chassis.
 */
public interface SwerveConstantsIF {

    /**
     * Converts ChassisSpeeds into SwerveModuleState (Array)
     * @param speeds ChassisSpeeds, the calculated speed of the robot.
     * @return SwerveModuleState, in an array according to ChassisSpeeds parameter.
     */
    SwerveModuleState[] toSwerveModuleStates(ChassisSpeeds speeds);

    /**
     * Converts SwerveModuleStates into ChassisSpeeds
     * @param swerveModuleStates all currently active swervemodulestates
     * @return ChassisSpeeds, the calculated speed of the chassis.
     */
    ChassisSpeeds toChassisSpeeds(SwerveModuleState... swerveModuleStates);

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
    SwerveDriveKinematics getSwerveDriveKinematics();

    double  maxRotationalVelocity();

}
