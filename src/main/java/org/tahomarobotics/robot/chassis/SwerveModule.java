package org.tahomarobotics.robot.chassis;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.LoggerManager;
import org.tahomarobotics.robot.util.MotorUtil;

public class SwerveModule {
    private static final int CAN_TIMEOUT_MS = 500;

    public record SwerveConfiguration(String name, RobotMap.SwerveModulePorts ports, double referenceAngle) {}

    private final String name;
    private final TalonFX driveMotor;
    private final CANSparkMax steerMotor;
    private final CANCoder steerEncoder;
    private SparkMaxPIDController steerController;
    private int resetIteration = ChassisConstants.ENCODER_RESET_ITERATIONS;

    private final PIDController drivePIDController = new PIDController(0, 0, 0);
    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0, ChassisConstants.kV_DRIVE);

    private SwerveModuleState state = new SwerveModuleState();

    public static void checkCtreError(ErrorCode errorCode, String message) {
        if (errorCode != ErrorCode.OK) {
            DriverStation.reportError(String.format("%s: %s", message, errorCode.toString()), false);
        }
    }

    public SwerveModule(SwerveConfiguration configuration) {
        this(configuration.name, configuration.ports, configuration.referenceAngle);
    }

    public SwerveModule(String name, RobotMap.SwerveModulePorts ports, double referenceAngle) {
        this.name = name;
        driveMotor = setupDriveMotor(ports.drive());
        steerMotor = setupSteerMotor(ports.steer());
        steerEncoder = setupSteerEncoder(ports.encoder(), referenceAngle);
    }

    private TalonFX setupDriveMotor(int driveMotorId) {
        TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
        motorConfiguration.voltageCompSaturation = ChassisConstants.REFERENCE_VOLTAGE;
        motorConfiguration.supplyCurrLimit.currentLimit = ChassisConstants.DRIVE_CURRENT_LIMIT;
        motorConfiguration.statorCurrLimit.currentLimit = ChassisConstants.DRIVE_ACCEL_CURRENT_LIMIT;
        motorConfiguration.supplyCurrLimit.enable = true;
        motorConfiguration.statorCurrLimit.enable = true;
        TalonFX motor = new TalonFX(driveMotorId);

        checkCtreError(motor.configAllSettings(motorConfiguration), "Failed to set current limit for drive motor " + driveMotorId);
        motor.enableVoltageCompensation(true);
        motor.setNeutralMode(NeutralMode.Brake);
        motor.setInverted(TalonFXInvertType.Clockwise);
        motor.setSensorPhase(true);

        MotorUtil.reduceRateGeneralStatus(motor);

        return motor;
    }

    private CANSparkMax setupSteerMotor(int steerMotorId) {
        CANSparkMax motor = new CANSparkMax(steerMotorId, CANSparkMaxLowLevel.MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        steerController = motor.getPIDController();
        for (int i = 0; i < 15; i++) {
            if (setupSteerConfig(motor)) {
                LoggerManager.log("Successfully setup steer motor for " + name);
                break;
            }
            LoggerManager.log("Retrying setting up steer motor for " + name);
        }

        // Reduce CAN status frame rates
        MotorUtil.reduceRateGeneralStatus(motor);
        motor.burnFlash();
        return motor;
    }
    private boolean setupSteerConfig (CANSparkMax motor) {
        // Idle mode
        REVLibError rtnCode = motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        if (rtnCode != REVLibError.kOk) {
            LoggerManager.error("Failed to setup idle mode for steer motor " + rtnCode);
            return false;
        }

        // Inversion
        motor.setInverted(true);
        if (!motor.getInverted()) {
            LoggerManager.error("Failed to setup motor inversion for steer motor ");
            return false;
        }
        rtnCode = motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        if (rtnCode != REVLibError.kOk) {
            LoggerManager.error("Failed to setup brake mode for steer motor" + rtnCode);
            return false;
        }


        //PID
        rtnCode = steerController.setP(0.3);
        if (rtnCode != REVLibError.kOk) {
            LoggerManager.error("Failed to setup P for steer PID" + rtnCode);
            return false;
        }
        rtnCode = steerController.setI(0.0);
        if (rtnCode != REVLibError.kOk) {
            LoggerManager.error("Failed to setup I for steer PID" + rtnCode);
            return false;
        }
        rtnCode = steerController.setD(0.1);
        if (rtnCode != REVLibError.kOk) {
            LoggerManager.error("Failed to setup D for steer PID" + rtnCode);
            return false;
        }

        rtnCode = motor.enableVoltageCompensation(ChassisConstants.REFERENCE_VOLTAGE);
        if (rtnCode != REVLibError.kOk) {
            LoggerManager.error("Failed to enable VoltageCompensation for steer" + rtnCode);
            return false;
        }
        rtnCode = motor.setSmartCurrentLimit((int) ChassisConstants.STEER_CURRENT_LIMIT);
        if (rtnCode != REVLibError.kOk) {
            LoggerManager.error("Failed to set current limit for steer" + rtnCode);
            return false;
        }
        return true;
    }


    private CANCoder setupSteerEncoder(int steerEncoderId, double referenceAngle) {
        CANCoderConfiguration config = new CANCoderConfiguration();
        config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        config.magnetOffsetDegrees = Math.toDegrees(referenceAngle);
        config.sensorDirection = false;

        CANCoder encoder = new CANCoder(steerEncoderId);
        checkCtreError(encoder.configAllSettings(config, CAN_TIMEOUT_MS), "Failed to configure CANCoder " + steerEncoderId);

        checkCtreError(encoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10, CAN_TIMEOUT_MS),
                "Failed to configure CANCoder update rate " + steerEncoderId);

        return encoder;
    }

    private double getAbsoluteAngle() {
        double angle = Math.toRadians(steerEncoder.getAbsolutePosition());
        angle %= 2.0 * Math.PI;
        if (angle < 0.0) {
            angle += 2.0 * Math.PI;
        }

        return angle;
    }

    /* From SDS with Modifications */
    public void setReferenceAngle(double referenceAngle) {

        double currentAngle = getSteerAngle();

        // Reset the NEO's encoder periodically when the module is not rotating.
        // Sometimes (~5% of the time) when we initialize, the absolute encoder isn't fully set up, and we don't
        // end up getting a good reading. If we reset periodically this won't matter anymore.
        if (Math.abs(getSteerVelocity()) < ChassisConstants.ENCODER_RESET_MAX_ANGULAR_VELOCITY) {
            if (++resetIteration >= ChassisConstants.ENCODER_RESET_ITERATIONS) {
                resetIteration = 0;
                // read degrees from CANcoder
                double absoluteAngle = getAbsoluteAngle();
                // System.out.println(name + " angle:" + Units.radiansToDegrees(absoluteAngle));
                steerMotor.getEncoder().setPosition(absoluteAngle / ChassisConstants.STEER_POSITION_COEFFICIENT);
                currentAngle = absoluteAngle;
            }
        } else {
            resetIteration = 0;
        }

        double currentAngleMod = currentAngle % (2.0 * Math.PI);
        if (currentAngleMod < 0.0) {
            currentAngleMod += 2.0 * Math.PI;
        }

        double adjustedReferenceAngle = referenceAngle + currentAngle - currentAngleMod;
        if (referenceAngle - currentAngleMod > Math.PI) {
            adjustedReferenceAngle -= 2.0 * Math.PI;
        } else if (referenceAngle - currentAngleMod < -Math.PI) {
            adjustedReferenceAngle += 2.0 * Math.PI;
        }

        steerController.setReference(adjustedReferenceAngle / ChassisConstants.STEER_POSITION_COEFFICIENT,
                                           CANSparkMax.ControlType.kPosition);
    }

    public double getVelocity() {
        return driveMotor.getSelectedSensorVelocity() * ChassisConstants.DRIVE_VELOCITY_COEFFICIENT;
    }

    public double getSteerAngle() {
        return steerMotor.getEncoder().getPosition() * ChassisConstants.STEER_POSITION_COEFFICIENT;
    }

    private double getSteerVelocity() {
        return steerMotor.getEncoder().getVelocity() * ChassisConstants.STEER_POSITION_COEFFICIENT * 10;
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return RobotBase.isReal() ? new SwerveModuleState(getVelocity(), new Rotation2d(getSteerAngle())) : state;
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        // This code is speculative as the documentation and examples on is non-existent
        return new SwerveModulePosition(steerEncoder.getPosition(), new Rotation2d(getSteerAngle()));
    }



    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        double steerAngle = getSteerAngle();

        // Optimize the reference state to avoid spinning further than 90 degrees
        state = SwerveModuleState.optimize(desiredState, new Rotation2d(steerAngle));

        // Calculate the drive output from the drive PID controller.
        final double driveOutput = drivePIDController.calculate(getVelocity(), state.speedMetersPerSecond);
        final double driveFeedforward = this.driveFeedforward.calculate(state.speedMetersPerSecond);

        setDriveVoltage(driveOutput + driveFeedforward);
        setReferenceAngle(state.angle.getRadians());
    }

    public void setDriveVoltage(double voltage) {
        driveMotor.set(TalonFXControlMode.PercentOutput, voltage / ChassisConstants.REFERENCE_VOLTAGE);
    }
}
