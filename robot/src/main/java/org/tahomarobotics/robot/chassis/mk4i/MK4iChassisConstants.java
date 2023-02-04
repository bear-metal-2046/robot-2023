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
package org.tahomarobotics.robot.chassis.mk4i;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.chassis.ChassisConstantsIF;
import org.tahomarobotics.robot.chassis.SwerveModuleIF;

import java.util.List;

/**
 * MK4I Chassis Configuration
 * // TODO: 1/31/2023 Change Chassis Specific Configuration to their own classes. However keep SwerveModule related configuration the same.
 */
public final class MK4iChassisConstants implements ChassisConstantsIF {



    public static final double TRACK_WIDTH = Units.inchesToMeters(23.5);
    public static final double WHEELBASE = Units.inchesToMeters(23.5);
    private static final double HALF_TRACK_WIDTH = TRACK_WIDTH / 2;
    private static final double HALF_WHEELBASE = WHEELBASE / 2;


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
            / Math.hypot(HALF_TRACK_WIDTH, HALF_WHEELBASE);


    //The Acceleration limiters for translation
    public static final double TRANSLATION_LIMIT = 9.0;

    //The Acceleration limiters for rotation
    public static final double ROTATION_LIMIT = TRANSLATION_LIMIT / Math.hypot(HALF_TRACK_WIDTH, HALF_WHEELBASE);


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

    private static final Translation2d FRONT_LEFT_OFFSET  = new Translation2d( HALF_WHEELBASE,  HALF_TRACK_WIDTH);
    private static final Translation2d FRONT_RIGHT_OFFSET = new Translation2d( HALF_WHEELBASE, -HALF_TRACK_WIDTH);
    private static final Translation2d REAR_LEFT_OFFSET   = new Translation2d(-HALF_WHEELBASE,  HALF_TRACK_WIDTH);
    private static final Translation2d REAR_RIGHT_OFFSET  = new Translation2d(-HALF_WHEELBASE, -HALF_TRACK_WIDTH);

    /**
     * The current swerveConstants must be defined in order for Chassis to configure properly.
     * These constants classes should not vary per robot, and are determined by the swerve module.
     * It is possible that in the future there may also be a "RobotConstant" class that IS robot
     * specific. However, at this time that is not needed.
     */
    @Override
    public List<SwerveModuleIF> createSwerveModules(Double  angularOffsets[]) {
        return List.of(
                new MK4iSwerveModule("Front-Left",  RobotMap.FRONT_LEFT_MOD,  FRONT_LEFT_OFFSET,  angularOffsets[0]),
                new MK4iSwerveModule("Front-Right", RobotMap.FRONT_RIGHT_MOD, FRONT_RIGHT_OFFSET, angularOffsets[1]),
                new MK4iSwerveModule("Back-Left",   RobotMap.BACK_LEFT_MOD,   REAR_LEFT_OFFSET,   angularOffsets[2]),
                new MK4iSwerveModule("Back-Right",  RobotMap.BACK_RIGHT_MOD,  REAR_RIGHT_OFFSET,  angularOffsets[3])
        );
    }

    @Override
    public double maxAttainableMps() {
        return MAX_VELOCITY_MPS;
    }

    @Override
    public double maxRotationalVelocity() {
        return MAX_ANGULAR_VELOCITY_RPS;
    }

    @Override
    public double accelerationLimit() {
        return TRANSLATION_LIMIT;
    }

    @Override
    public double angularAccelerationLimit() {
        return ROTATION_LIMIT;
    }
}
