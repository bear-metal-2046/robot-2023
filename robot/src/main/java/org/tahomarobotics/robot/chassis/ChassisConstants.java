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

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.DoubleProperty;

/**
 * Constants Class
 * @implNote May be replaced by YAML Configuration.
 */
public final class ChassisConstants {
    //TODO measuringggg
    public static final double CHASSIS_WIDTH = 0.5969;
    //TODO more measuringggggg
    public static final double CHASSIS_WHEELBASE = 0.5969;

    public static final SwerveModule.SwerveConfiguration FRONT_LEFT_SWERVE_CONFIG = new SwerveModule.SwerveConfiguration(
            "FRONT_LEFT", RobotMap.FRONT_LEFT_MOD,
            new DoubleProperty("FL", 0.0));

    public static final SwerveModule.SwerveConfiguration FRONT_RIGHT_SWERVE_CONFIG = new SwerveModule.SwerveConfiguration(
            "FRONT_RIGHT", RobotMap.FRONT_RIGHT_MOD,
            new DoubleProperty("FR", 0.0));

    public static final SwerveModule.SwerveConfiguration BACK_LEFT_SWERVE_CONFIG = new SwerveModule.SwerveConfiguration(
            "BACK_LEFT", RobotMap.BACK_LEFT_MOD,
            new DoubleProperty("BL", 0.0));

    public static final SwerveModule.SwerveConfiguration BACK_RIGHT_SWERVE_CONFIG = new SwerveModule.SwerveConfiguration(
            "BACK_RIGHT", RobotMap.BACK_RIGHT_MOD,
            new DoubleProperty("BR", 0.0));

    private static final double X_OFFSET = CHASSIS_WIDTH / 2;
    private static final double Y_OFFSET = CHASSIS_WHEELBASE / 2;

    public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(X_OFFSET, Y_OFFSET),
            new Translation2d(X_OFFSET, -Y_OFFSET),
            new Translation2d(-X_OFFSET, Y_OFFSET),
            new Translation2d(-X_OFFSET, -Y_OFFSET));

    //IN METERS
    public static final double WHEEL_DIAMETER = 0.10033;
    public static final double WHEEL_RADIUS = WHEEL_DIAMETER / 2;
    public static final double WHEEL_CIRCUMFERENCE = 2 * Math.PI * WHEEL_RADIUS;
    public static final double DRIVE_REDUCTION_MK4I_L2 = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
    public static final double STEER_REDUCTION = (14.0 / 50.0) * (10.0 / 60.0);
    public static final double REFERENCE_VOLTAGE = 12.0;
    public static final double DRIVE_CURRENT_LIMIT = 80.0;
    public static final double DRIVE_ACCEL_RPM_LIMIT = 2016; //idk what to put for RPM CAUSEEEEE AHHHHHH
    public static final double STEER_CURRENT_LIMIT = 20.0;

    public static final DCMotor SWERVE_DRIVE_MOTOR = DCMotor.getNEO(1);

    // Hypothetical max velocity of the drive motor in meters per second
    public static final double MAX_VELOCITY_MPS = SWERVE_DRIVE_MOTOR.freeSpeedRadPerSec * DRIVE_REDUCTION_MK4I_L2
            * WHEEL_RADIUS;

    // Hypothetical max velocity of spinning chassis in RADIANS per second
    public static final double MAX_ANGULAR_VELOCITY_RPS = MAX_VELOCITY_MPS
            / Math.hypot(CHASSIS_WIDTH / 2.0, CHASSIS_WHEELBASE / 2.0);


    //The Acceleration limiters for translation
    public static final double TRANSLATION_LIMIT = 9.0;

    //The Acceleration limiters for rotation
    public static final double ROTATION_LIMIT = TRANSLATION_LIMIT / (CHASSIS_WIDTH / 2.0);


    // For the times when you don't want your max velocity to be max (0.0 - 1.0)
    public static final double VELOCITY_MULTIPLIER = 1.0;

    public static final double DRIVE_VELOCITY_COEFFICIENT = Math.PI * WHEEL_DIAMETER * DRIVE_REDUCTION_MK4I_L2 / 4096 * 10;
    
    public static final double STEER_POSITION_COEFFICIENT = 2.0 * Math.PI * STEER_REDUCTION / 4096;
    
    public static final double ENCODER_RESET_MAX_ANGULAR_VELOCITY = Math.toRadians(0.5);
    
    public static final int ENCODER_RESET_ITERATIONS = 500;

    public static final double MASS = Units.lbsToKilograms(55.85);

    // volts per mps
    public static final double kV_DRIVE = 1.0 / DRIVE_REDUCTION_MK4I_L2
            / (SWERVE_DRIVE_MOTOR.KvRadPerSecPerVolt * WHEEL_RADIUS);
    
    // volts per mps^2
    // ohms * meter * kg / Nm * Amps = volts * kg / N = volts * kg / (kg m/s^2) =
    // volts / m/s2
    public static final double kA_DRIVE = SWERVE_DRIVE_MOTOR.rOhms * WHEEL_RADIUS * MASS
            / (DRIVE_REDUCTION_MK4I_L2 * SWERVE_DRIVE_MOTOR.KtNMPerAmp);

}
