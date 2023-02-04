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
package org.tahomarobotics.robot.chassis.config;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.chassis.module.MAXSwerveModule;

/**
 * Constants Class
 * @implNote May be replaced by YAML Configuration.
 */
public final class REVMaxConstants implements SwerveConstantsIF{
    static double FRONT_LEFT_OFFSET = 3.672287;
    static double FRONT_RIGHT_OFFSET = 1.048095;
    static double BACK_LEFT_OFFSET = 4.015447;
    static double BACK_RIGHT_OFFSET = 1.140168;


    //TODO measuringggg
    public static final double CHASSIS_WIDTH = 0.5969;
    //TODO more measuringggggg
    public static final double CHASSIS_WHEELBASE = 0.5969;

    public static final MAXSwerveModule.SwerveConfiguration FRONT_LEFT_SWERVE_CONFIG = new MAXSwerveModule.SwerveConfiguration(
            "FRONT_LEFT", RobotMap.FRONT_LEFT_MOD, FRONT_LEFT_OFFSET);

    public static final MAXSwerveModule.SwerveConfiguration FRONT_RIGHT_SWERVE_CONFIG = new MAXSwerveModule.SwerveConfiguration(
            "FRONT_RIGHT", RobotMap.FRONT_RIGHT_MOD, FRONT_RIGHT_OFFSET);

    public static final MAXSwerveModule.SwerveConfiguration BACK_LEFT_SWERVE_CONFIG = new MAXSwerveModule.SwerveConfiguration(
            "BACK_LEFT", RobotMap.BACK_LEFT_MOD, BACK_LEFT_OFFSET);

    public static final MAXSwerveModule.SwerveConfiguration BACK_RIGHT_SWERVE_CONFIG = new MAXSwerveModule.SwerveConfiguration(
            "BACK_RIGHT", RobotMap.BACK_RIGHT_MOD, BACK_RIGHT_OFFSET);

    private static final double X_OFFSET = CHASSIS_WIDTH / 2;
    private static final double Y_OFFSET = CHASSIS_WHEELBASE / 2;

    public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(X_OFFSET, Y_OFFSET),
            new Translation2d(X_OFFSET, -Y_OFFSET),
            new Translation2d(-X_OFFSET, Y_OFFSET),
            new Translation2d(-X_OFFSET, -Y_OFFSET));

    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int DRIVING_MOTOR_PINION_TEETH = 14;
    public static final double WHEEL_DIAMETER = 0.0762;
    public static final double WHEEL_RADIUS = WHEEL_DIAMETER / 2;

    public static final double WHEEL_CIRCUMFERENCE = 2 * Math.PI * WHEEL_RADIUS;

    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double DRIVE_REDUCTION = (DRIVING_MOTOR_PINION_TEETH * 15) / (45.0 * 22);

    public static final double DRIVE_ENCODER_POSITION_FACTOR = (WHEEL_DIAMETER * Math.PI) * DRIVE_REDUCTION;
    public static final double DRIVE_ENCODER_VELOCITY_FACTOR = ((WHEEL_DIAMETER * Math.PI) * DRIVE_REDUCTION) / 60.0;
    public static final double STEER_ENCODER_POSITION_FACTOR = (2 * Math.PI);
    public static final double STEER_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60;

    public static final double REFERENCE_VOLTAGE = 12.0;
    public static final double DRIVE_CURRENT_LIMIT = 50.0;
    public static final double STEER_CURRENT_LIMIT = 20.0;

    public static final DCMotor SWERVE_DRIVE_MOTOR = DCMotor.getNEO(1);

    public static final double DRIVE_MOTOR_FREE_SPEED_RPS = 5676 / 60d;
    public static final double DRIVE_WHEEL_FREE_SPEED_RPS = DRIVE_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE * DRIVE_REDUCTION;


    // Hypothetical max velocity of the drive motor in meters per second
    public static final double MAX_VELOCITY_MPS = SWERVE_DRIVE_MOTOR.freeSpeedRadPerSec * DRIVE_REDUCTION
            * WHEEL_RADIUS;

    // Hypothetical max velocity of spinning chassis in RADIANS per second
    public static final double MAX_ANGULAR_VELOCITY_RPS = MAX_VELOCITY_MPS
            / Math.hypot(CHASSIS_WIDTH / 2.0, CHASSIS_WHEELBASE / 2.0);


    //The Acceleration limiters for translation
    public static final double TRANSLATION_LIMIT = 9.0;

    //The Acceleration limiters for rotation
    public static final double ROTATION_LIMIT = TRANSLATION_LIMIT / (CHASSIS_WIDTH / 2.0);

    public static final double MASS = Units.lbsToKilograms(55.85);

    // volts per mps
    public static final double kV_DRIVE = 1.0 / DRIVE_REDUCTION
            / (SWERVE_DRIVE_MOTOR.KvRadPerSecPerVolt * WHEEL_RADIUS);

    // volts per mps^2
    // ohms * meter * kg / Nm * Amps = volts * kg / N = volts * kg / (kg m/s^2) =
    // volts / m/s2
    public static final double kA_DRIVE = SWERVE_DRIVE_MOTOR.rOhms * WHEEL_RADIUS * MASS
            / (DRIVE_REDUCTION * SWERVE_DRIVE_MOTOR.KtNMPerAmp);

    @Override
    public SwerveModuleState[] toSwerveModuleStates(ChassisSpeeds speeds) {
        return SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(speeds);
    }

    @Override
    public ChassisSpeeds toChassisSpeeds(SwerveModuleState... swerveModuleStates) {
        return SWERVE_DRIVE_KINEMATICS.toChassisSpeeds(swerveModuleStates);
    }

    @Override
    public double maxAttainableMps() {
        return MAX_VELOCITY_MPS;
    }

    @Override
    public double accelerationLimit() {
        return TRANSLATION_LIMIT;
    }

    @Override
    public double angularAccelerationLimit() {
        return ROTATION_LIMIT;
    }

    @Override
    public SwerveDriveKinematics getSwerveDriveKinematics() {
        return SWERVE_DRIVE_KINEMATICS;
    }

    @Override
    public double maxRotationalVelocity() {
        return MAX_ANGULAR_VELOCITY_RPS;
    }
}
