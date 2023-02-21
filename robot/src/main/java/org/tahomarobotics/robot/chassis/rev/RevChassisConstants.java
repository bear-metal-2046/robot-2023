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
package org.tahomarobotics.robot.chassis.rev;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.chassis.ChassisConstantsIF;
import org.tahomarobotics.robot.chassis.SwerveModuleIF;
import org.tahomarobotics.robot.util.SparkMaxConfig;

import java.util.List;

/**
 * Constants Class
 * @implNote May be replaced by YAML Configuration.
 */
public final class RevChassisConstants implements ChassisConstantsIF {

    public static final double TRACK_WIDTH = Units.inchesToMeters(24.5);
    public static final double WHEELBASE = Units.inchesToMeters(24.5);
    private static final double HALF_TRACK_WIDTH = TRACK_WIDTH / 2;
    private static final double HALF_WHEELBASE = WHEELBASE / 2;

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

    public static final int DRIVE_CURRENT_LIMIT = 50;
    public static final int STEER_CURRENT_LIMIT = 20;

    public static final DCMotor SWERVE_DRIVE_MOTOR = DCMotor.getNEO(1);

    public static final double DRIVE_MOTOR_FREE_SPEED_RPS = 5676 / 60d;
    public static final double DRIVE_WHEEL_FREE_SPEED_RPS = DRIVE_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE * DRIVE_REDUCTION;


    // Hypothetical max velocity of the drive motor in meters per second
    public static final double MAX_VELOCITY_MPS = SWERVE_DRIVE_MOTOR.freeSpeedRadPerSec * DRIVE_REDUCTION
            * WHEEL_RADIUS;

    // Hypothetical max velocity of spinning chassis in RADIANS per second
    public static final double MAX_ANGULAR_VELOCITY_RPS = MAX_VELOCITY_MPS
            / Math.hypot(HALF_TRACK_WIDTH, HALF_WHEELBASE);


    //The Acceleration limiters for translation
    public static final double TRANSLATION_LIMIT = 9.0;

    //The Acceleration limiters for rotation
    public static final double ROTATION_LIMIT = TRANSLATION_LIMIT/ Math.hypot(HALF_TRACK_WIDTH, HALF_WHEELBASE);

    public static final double MASS = Units.lbsToKilograms(55.85);

    // volts per mps
    public static final double kV_DRIVE = 1.0 / DRIVE_REDUCTION
            / (SWERVE_DRIVE_MOTOR.KvRadPerSecPerVolt * WHEEL_RADIUS);

    // volts per mps^2
    // ohms * meter * kg / Nm * Amps = volts * kg / N = volts * kg / (kg m/s^2) =
    // volts / m/s2
    public static final double kA_DRIVE = SWERVE_DRIVE_MOTOR.rOhms * WHEEL_RADIUS * MASS
            / (DRIVE_REDUCTION * SWERVE_DRIVE_MOTOR.KtNMPerAmp);

    private static final Translation2d FRONT_LEFT_OFFSET  = new Translation2d( HALF_WHEELBASE,  HALF_TRACK_WIDTH);
    private static final Translation2d FRONT_RIGHT_OFFSET = new Translation2d( HALF_WHEELBASE, -HALF_TRACK_WIDTH);
    private static final Translation2d REAR_LEFT_OFFSET   = new Translation2d(-HALF_WHEELBASE,  HALF_TRACK_WIDTH);
    private static final Translation2d REAR_RIGHT_OFFSET  = new Translation2d(-HALF_WHEELBASE, -HALF_TRACK_WIDTH);

    @Override
    public List<SwerveModuleIF> createSwerveModules(Double  angularOffsets[]) {
        return List.of(
                new RevSwerveModule("Front-Left",  RobotMap.FRONT_LEFT_MOD,  FRONT_LEFT_OFFSET,  angularOffsets[0]),
                new RevSwerveModule("Front-Right", RobotMap.FRONT_RIGHT_MOD, FRONT_RIGHT_OFFSET, angularOffsets[1]),
                new RevSwerveModule("Back-Left",   RobotMap.BACK_LEFT_MOD,   REAR_LEFT_OFFSET,   angularOffsets[2]),
                new RevSwerveModule("Back-Right",  RobotMap.BACK_RIGHT_MOD,  REAR_RIGHT_OFFSET,  angularOffsets[3]));
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
    public double maxRotationalVelocity() {
        return MAX_ANGULAR_VELOCITY_RPS;
    }


    public static SparkMaxConfig createDriveConfig(int id) {
        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.canId = id;
        cfg.currentLimit = DRIVE_CURRENT_LIMIT;
        cfg.positionConversionFactor = DRIVE_ENCODER_POSITION_FACTOR;
        cfg.velocityConversionFactor = DRIVE_ENCODER_VELOCITY_FACTOR;
        return cfg;
    }
    public static SparkMaxConfig createSteerConfig(int id, double offset) {
        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.canId = id;
        cfg.currentLimit = STEER_CURRENT_LIMIT;
        cfg.positionConversionFactor = STEER_ENCODER_POSITION_FACTOR;
        cfg.velocityConversionFactor = STEER_ENCODER_VELOCITY_FACTOR;
        cfg.encoderInverted = true;
        cfg.encoderOffset = offset;
        cfg.kP = 0.75;
        cfg.wrapEnabled = true;
        return cfg;
    }
}
