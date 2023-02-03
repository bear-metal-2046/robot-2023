package org.tahomarobotics.robot.chassis.config;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * Directly imported from the MAXSwerve Example project on GitHub.
 * @link https://github.com/REVrobotics/MAXSwerve-Java-Template
 * // TODO: 1/31/2023   To be replaced with Zachs MAXSwerve Constants code
 */
public class REVMAXConstants implements SwerveConstantsIF {
    public static final class DriveConstants {
        //TODO A lot of things contained within this class will need to be modified on a robot-by-robot basis.

        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static final double kMaxSpeedMetersPerSecond = 4.8;

        // Chassis configuration
        public static final double kTrackWidth = Units.inchesToMeters(26.5);
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(26.5);
        // Distance between front and back wheels on robot

        public static final double kMaxAngularSpeed = kMaxSpeedMetersPerSecond
            / Math.hypot(kTrackWidth / 2.0, kWheelBase / 2.0);

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        // Angular offsets of the modules relative to the chassis in radians
        public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
        public static final double kFrontRightChassisAngularOffset = 0;
        public static final double kBackLeftChassisAngularOffset = Math.PI;
        public static final double kBackRightChassisAngularOffset = Math.PI / 2;

        //The Acceleration limiters for translation
        public static final double TRANSLATION_LIMIT = 9.0;

        //The Acceleration limiters for rotation
        public static final double ROTATION_LIMIT = TRANSLATION_LIMIT / (kWheelBase / 2.0);

        // SPARK MAX CAN IDs
        /* TODO Put all ports under specific class.
        public static final int kFrontLeftDrivingCanId = 11;
        public static final int kRearLeftDrivingCanId = 13;
        public static final int kFrontRightDrivingCanId = 15;
        public static final int kRearRightDrivingCanId = 17;

        public static final int kFrontLeftTurningCanId = 10;
        public static final int kRearLeftTurningCanId = 12;
        public static final int kFrontRightTurningCanId = 14;
        public static final int kRearRightTurningCanId = 16;
         */

        public static final boolean kGyroReversed = false;
    }

    public static final class ModuleConstants {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
        // This changes the drive speed of the module (a pinion gear with more teeth will result in a
        // robot that drives faster).
        public static final int kDrivingMotorPinionTeeth = 14;

        // Invert the turning encoder, since the output shaft rotates in the opposite direction of
        // the steering motor in the MAXSwerve Module.
        public static final boolean kTurningEncoderInverted = true;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
        public static final double kWheelDiameterMeters = 0.0762;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
        public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
                / kDrivingMotorReduction;

        public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
                / kDrivingMotorReduction; // meters
        public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
                / kDrivingMotorReduction) / 60.0; // meters per second

        public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
        public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

        public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
        public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

        public static final double kDrivingP = 0.04;
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 0;
        public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
        public static final double kDrivingMinOutput = -1;
        public static final double kDrivingMaxOutput = 1;

        public static final double kTurningP = 1;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0;
        public static final double kTurningFF = 0;
        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput = 1;

        public static final CANSparkMax.IdleMode kDrivingMotorIdleMode = CANSparkMax.IdleMode.kBrake;
        public static final CANSparkMax.IdleMode kTurningMotorIdleMode = CANSparkMax.IdleMode.kBrake;

        public static final int kDrivingMotorCurrentLimit = 50; // amps
        public static final int kTurningMotorCurrentLimit = 20; // amps

        public static final double CHASSIS_WIDTH = 0.5969;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 5676;
    }

    @Override
    public SwerveModuleState[] toSwerveModuleStates(ChassisSpeeds speeds) {
        return DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    }

    @Override
    public ChassisSpeeds toChassisSpeeds(SwerveModuleState... swerveModuleStates) {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(swerveModuleStates);
    }

    @Override
    public double maxAttainableMps() {
        //Chance 1.0 to something lower if you want your max speed to be a *little bit less*
        return DriveConstants.kMaxSpeedMetersPerSecond * 1.0;
    }

    @Override
    public SwerveDriveKinematics getSwerveDriveKinematics() {
        return DriveConstants.kDriveKinematics;
    }

    @Override
    public double maxRotationalVelocity() {
        return DriveConstants.kMaxAngularSpeed;
    }

    @Override
    public double angularAccelerationLimit() {
        return DriveConstants.ROTATION_LIMIT;
    }

    @Override
    public double accelerationLimit() {
        return DriveConstants.TRANSLATION_LIMIT;
    }
}