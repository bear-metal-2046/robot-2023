package org.tahomarobotics.robot.chassis;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import org.tahomarobotics.robot.RobotMap;

@Deprecated(since = "Values used from 2022; Please Update")
public final class ChassisConstants {
    static double FRONT_LEFT_OFFSET = Math.toRadians(0);
    static double FRONT_RIGHT_OFFSET = Math.toRadians(0);
    static double BACK_LEFT_OFFSET = Math.toRadians(0);
    static double BACK_RIGHT_OFFSET = Math.toRadians(0);

    static double FL_SPARE_OFFSET = Math.toRadians(0);
    static double FR_SPARE_OFFSET = Math.toRadians(0);
    static double BL_SPARE_OFFSET = Math.toRadians(0);
    static double BR_SPARE_OFFSET = Math.toRadians(0);

    public static final double CHASSIS_WIDTH = 0.5969;
    public static final double CHASSIS_WHEELBASE = 0.5969;

    public static final SwerveModule.SwerveConfiguration L_F_SWERVE_CONFIG = new SwerveModule.SwerveConfiguration(
            "L_FWD", RobotMap.FRONT_LEFT_MOD, FRONT_LEFT_OFFSET);
    // RobotMap.SPARE_DRIVE, RobotMap.SPARE_STEER, RobotMap.SPARE_ENCODER,
    // BR_SPARE_OFFSET);

    public static final SwerveModule.SwerveConfiguration R_F_SWERVE_CONFIG = new SwerveModule.SwerveConfiguration(
            "R_FWD", RobotMap.FRONT_RIGHT_MOD, FRONT_RIGHT_OFFSET);
    // RobotMap.SPARE_DRIVE, RobotMap.SPARE_STEER, RobotMap.SPARE_ENCODER,
    // BR_SPARE_OFFSET);

    public static final SwerveModule.SwerveConfiguration L_B_SWERVE_CONFIG = new SwerveModule.SwerveConfiguration(
            "L_AFT", RobotMap.BACK_LEFT_MOD, BACK_LEFT_OFFSET);
    // RobotMap.SPARE_DRIVE, RobotMap.SPARE_STEER, RobotMap.SPARE_ENCODER,
    // BR_SPARE_OFFSET);

    public static final SwerveModule.SwerveConfiguration R_B_SWERVE_CONFIG = new SwerveModule.SwerveConfiguration(
            "R_AFT", RobotMap.BACK_RIGHT_MOD, BACK_RIGHT_OFFSET);
    // RobotMap.SPARE_DRIVE, RobotMap.SPARE_STEER, RobotMap.SPARE_ENCODER,
    // BR_SPARE_OFFSET);

    private static final double X_OFFSET = CHASSIS_WIDTH / 2;
    private static final double Y_OFFSET = CHASSIS_WHEELBASE / 2;

    public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(X_OFFSET, Y_OFFSET),
            new Translation2d(X_OFFSET, -Y_OFFSET),
            new Translation2d(-X_OFFSET, Y_OFFSET),
            new Translation2d(-X_OFFSET, -Y_OFFSET));

    public static final double WHEEL_DIAMETER = Units.inchesToMeters(3.95);
    public static final double WHEEL_RADIUS = WHEEL_DIAMETER / 2;
    public static final double DRIVE_REDUCTION_MK4_L1 = (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0);
    public static final double DRIVE_REDUCTION_MK4_L2 = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
    public static final double STEER_REDUCTION = (15.0 / 32.0) * (10.0 / 60.0);

    public static final double REFERENCE_VOLTAGE = 12.0;
    public static final double DRIVE_CURRENT_LIMIT = 80.0;
    public static final double DRIVE_ACCEL_CURRENT_LIMIT = 40.0;
    public static final double STEER_CURRENT_LIMIT = 20.0;

    public static final DCMotor SWERVE_DRIVE_MOTOR = DCMotor.getFalcon500(1);

    // Hypothetical max velocity of the drive motor in meters per second
    public static final double MAX_VELOCITY_MPS = SWERVE_DRIVE_MOTOR.freeSpeedRadPerSec * DRIVE_REDUCTION_MK4_L1
            * WHEEL_RADIUS;

    // Hypothetical max velocity of spinning chassis in RADIANS per second
    public static final double MAX_ANGULAR_VELOCITY_RPS = MAX_VELOCITY_MPS
            / Math.hypot(CHASSIS_WIDTH / 2.0, CHASSIS_WHEELBASE / 2.0);

    // For the times when you don't want your max velocity to be max (0.0 - 1.0)
    public static final double VELOCITY_MULTIPLIER = 1.0;

    public static final double DRIVE_VELOCITY_COEFFICIENT = Math.PI * WHEEL_DIAMETER * DRIVE_REDUCTION_MK4_L1 / 2048
            * 10;
    public static final double STEER_POSITION_COEFFICIENT = 2.0 * Math.PI * STEER_REDUCTION / 2048;
    public static final double ENCODER_RESET_MAX_ANGULAR_VELOCITY = Math.toRadians(0.5);
    public static final int ENCODER_RESET_ITERATIONS = 500;

    public static final double MASS = Units.lbsToKilograms(55.85);
    public static final double kInertia = 0;

    // volts per mps
    public static final double kV_DRIVE = 1.0 / DRIVE_REDUCTION_MK4_L1
            / (SWERVE_DRIVE_MOTOR.KvRadPerSecPerVolt * WHEEL_RADIUS);
    // volts per mps^2
    // ohms * meter * kg / Nm * Amps = volts * kg / N = volts * kg / (kg m/s^2) =
    // volts / m/s2
    public static final double kA_DRIVE = SWERVE_DRIVE_MOTOR.rOhms * WHEEL_RADIUS * MASS
            / (DRIVE_REDUCTION_MK4_L1 * SWERVE_DRIVE_MOTOR.KtNMPerAmp);

}
