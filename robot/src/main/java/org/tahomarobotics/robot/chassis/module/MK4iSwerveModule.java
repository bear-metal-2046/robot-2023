/**
 * Copyright 2023 Tahoma Robotics - http://tahomarobotics.org - Bear Metal 2046 FRC Team
 * <p>
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including without
 * limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
 * Software, and to permit persons to whom the Software is furnished to do so, subject to the following
 * conditions:
 * <p>
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions
 * of the Software.
 * <p>
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
 * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */
package org.tahomarobotics.robot.chassis.module;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.revrobotics.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.chassis.config.MK4iConstants;
import org.tahomarobotics.robot.util.DoubleProperty;

import java.util.function.Supplier;

/**
 * MK4iSwerveModuleClass
 * Serves as an object for each MK4i Swerve Module, abstracted via SwerveModuleBase.
 */
public class MK4iSwerveModule implements SwerveModuleIF {

    private static final Logger logger = LoggerFactory.getLogger(MK4iSwerveModule.class);

    private static final int CAN_TIMEOUT_MS = 500;

    @Override
    public void displayPosition() {
        SmartDashboard.putNumber(name + " Swerve Module Position", getSteerAngle());
    }

    public record SwerveConfiguration(String name, RobotMap.SwerveModulePorts ports, DoubleProperty offset) {
    }

    private final String name;
    private final CANSparkMax driveMotor;
    private final CANSparkMax steerMotor;
    private final CANCoder steerABSEncoder;
    private SparkMaxPIDController steerPIDController;
    private int resetIteration = MK4iConstants.ENCODER_RESET_ITERATIONS;

    private final PIDController drivePIDController = new PIDController(0, 0, 0);
    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0, MK4iConstants.kV_DRIVE);

    private SwerveModuleState state = new SwerveModuleState();
    private final DoubleProperty offset;

    public MK4iSwerveModule(SwerveConfiguration configuration) {
        this(configuration.name, configuration.ports, configuration.offset);
    }

    public MK4iSwerveModule(String name, RobotMap.SwerveModulePorts ports, DoubleProperty offset) {
        this.name = name;
        driveMotor = setupDriveMotor(ports.drive());
        steerMotor = setupSteerMotor(ports.steer());
        this.offset = offset;
        steerABSEncoder = setupSteerEncoder(ports.encoder());
    }


    private static void checkCtreError(ErrorCode errorCode, String message) {
        if (errorCode != ErrorCode.OK) {
            logger.error(String.format("%s: %s", message, errorCode.toString()));
        }
    }

    private CANSparkMax setupDriveMotor(int driveMotorId) {
        CANSparkMax motor = new CANSparkMax(driveMotorId, CANSparkMaxLowLevel.MotorType.kBrushless);

        double posConv = Math.PI * MK4iConstants.WHEEL_DIAMETER * MK4iConstants.DRIVE_REDUCTION_MK4I_L2;
        boolean settingsChanged = setupMotorConfig(
                motor,
                posConv,
                true
        );

        // Is not stored in flash and has to be set every time.
        motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100);
        motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
        motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20);

        if (settingsChanged) {
            logger.info("Flashing settings to drive motor for swerve module " + name);
            motor.burnFlash();
        }

        return motor;
    }

    private boolean kindaEqual(double a, double b) {
        return Math.abs(a - b) < 0.01;
    }

    private CANSparkMax setupSteerMotor(int steerMotorId) {
        CANSparkMax motor = new CANSparkMax(steerMotorId, CANSparkMaxLowLevel.MotorType.kBrushless);
        steerPIDController = motor.getPIDController();

        double posConv = 2 * Math.PI * MK4iConstants.STEER_REDUCTION;

        boolean settingsChanged = setupMotorConfig(
                motor,
                posConv,
                true
        ) |
                setSetting(
                        kindaEqual(steerPIDController.getP(), .3),
                        () -> steerPIDController.setP(.3),
                        "Failed to set P"
                ) |
                setSetting(
                        kindaEqual(steerPIDController.getI(), 0.0),
                        () -> steerPIDController.setI(0),
                        "Failed to set I"
                ) |
                setSetting(
                        kindaEqual(steerPIDController.getD(), .1),
                        () -> steerPIDController.setD(.1),
                        "Failed to set D"
                );

        // Reduce CAN status frame rates
        motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 10);
        motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
        motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 50);

        // No getter; Sets on other settings changed.
        motor.setSmartCurrentLimit((int) MK4iConstants.STEER_CURRENT_LIMIT);

        if (settingsChanged) {
            logger.warn("Flashing settings to steer motor for swerve module " + name);
            motor.burnFlash();
        }

        return motor;
    }

    private boolean setSetting(boolean isBueno, Supplier<REVLibError> set, String err) {
        if (!isBueno) {
            REVLibError rtnCode = set.get();
            if (rtnCode == REVLibError.kOk) {
                return true;
            }
            logger.error(err + " for motor on " + name + "\n\tError: " + rtnCode);
        }

        return false;
    }

    private boolean setupMotorConfig(CANSparkMax motor, double posConversion, boolean inverted) {
        double velConversion = posConversion / 60.0;
        boolean settingsChanged = setSetting(
                kindaEqual(motor.getEncoder().getPositionConversionFactor(), posConversion),
                () -> motor.getEncoder().setPositionConversionFactor(posConversion),
                "Failed to set position conversion factor"
        );

        settingsChanged |= setSetting(
                kindaEqual(motor.getEncoder().getVelocityConversionFactor(), velConversion),
                () -> motor.getEncoder().setVelocityConversionFactor(velConversion),
                "Failed to set velocity conversion factor"
        );

        settingsChanged |= setSetting(
                motor.getIdleMode() == CANSparkMax.IdleMode.kBrake,
                () -> motor.setIdleMode(CANSparkMax.IdleMode.kBrake),
                "Failed to set brake mode"
        );

        settingsChanged |= setSetting(
                motor.getInverted() == inverted,
                () -> {
                    motor.setInverted(inverted);
                    return REVLibError.kOk;
                },
                "Failed to set inversion"
        );

        settingsChanged |= setSetting(
                kindaEqual(motor.getVoltageCompensationNominalVoltage(), MK4iConstants.REFERENCE_VOLTAGE),
                () -> motor.enableVoltageCompensation(MK4iConstants.REFERENCE_VOLTAGE),
                "Failed to set voltage compensation"
        );

        return settingsChanged;
    }


    private CANCoder setupSteerEncoder(int steerEncoderId) {
        CANCoderConfiguration config = new CANCoderConfiguration();
        config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        config.magnetOffsetDegrees = Math.toDegrees(offset.getValue());
        config.sensorDirection = false;

        CANCoder encoder = new CANCoder(steerEncoderId);
        checkCtreError(encoder.configAllSettings(config, CAN_TIMEOUT_MS), "Failed to configure CANCoder " + steerEncoderId);

        checkCtreError(encoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10, CAN_TIMEOUT_MS),
                "Failed to configure CANCoder update rate " + steerEncoderId);

        return encoder;
    }

    @Override
    public void updateOffset() {
        steerABSEncoder.configMagnetOffset(Units.radiansToDegrees(offset.getValue()));
        setReferenceAngle(0);
    }

    @Override
    public void align() {
        offset.setValue(-getAbsoluteAngle());
        updateOffset();
        steerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        logger.info(name + " successfully calibrated! New Offset: " + -getAbsoluteAngle());
    }

    @Override
    public void zeroOffset() {
        steerABSEncoder.configMagnetOffset(0);
        steerMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    public double getAbsoluteAngle() {
        double angle = Math.toRadians(steerABSEncoder.getAbsolutePosition());
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
        if (Math.abs(getSteerVelocity()) < MK4iConstants.ENCODER_RESET_MAX_ANGULAR_VELOCITY) {
            if (++resetIteration >= MK4iConstants.ENCODER_RESET_ITERATIONS) {
                resetIteration = 0;
                // read degrees from CANcoder
                double absoluteAngle = getAbsoluteAngle();
                // System.out.println(name + " angle:" + Units.radiansToDegrees(absoluteAngle));
                steerMotor.getEncoder().setPosition(absoluteAngle);
                currentAngle = absoluteAngle;
            }
        } else {
            resetIteration = 0;
        }

        double currentAngleMod = currentAngle % (2.0 * Math.PI);
        if (currentAngleMod < 0.0) {
            currentAngleMod += 2.0 * Math.PI;
        }

        // The reference angle has the range [0, 2pi) but the Neo's encoder can go above that
        double adjustedReferenceAngle = referenceAngle + currentAngle - currentAngleMod;
        if (referenceAngle - currentAngleMod > Math.PI) {
            adjustedReferenceAngle -= 2.0 * Math.PI;
        } else if (referenceAngle - currentAngleMod < -Math.PI) {
            adjustedReferenceAngle += 2.0 * Math.PI;
        }

        steerPIDController.setReference(adjustedReferenceAngle, CANSparkMax.ControlType.kPosition);
    }

    @Override
    public double getVelocity() {
        return driveMotor.getEncoder().getVelocity();
    }

    public double getSteerAngle() {
        return steerMotor.getEncoder().getPosition();
    }

    private double getSteerVelocity() {
        return steerMotor.getEncoder().getVelocity();
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), new Rotation2d(getSteerAngle()));
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    @Override
    public SwerveModulePosition getPosition() {
        // This code is speculative as the documentation and examples on is non-existent
        return new SwerveModulePosition(driveMotor.getEncoder().getPosition(), new Rotation2d(getSteerAngle()));
    }

    public void driveMotor(double power) {
        driveMotor.set(power);
    }

    public void applyPowerToTheRotationMotorInOrderToTestItOutAndMakeSureItWorks(double power) {
        steerMotor.set(power);
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    @Override
    public void setDesiredState(SwerveModuleState desiredState) {
        double steerAngle = getSteerAngle();

        // Optimize the reference state to avoid spinning further than 90 degrees
        state = SwerveModuleState.optimize(desiredState, new Rotation2d(steerAngle));

        // Calculate the drive output from the drive PID controller.
        final double driveOutput = drivePIDController.calculate(getVelocity(), state.speedMetersPerSecond);
        final double driveFeedforward = this.driveFeedforward.calculate(state.speedMetersPerSecond);

        setDriveVoltage(driveOutput + driveFeedforward);
        SmartDashboard.putNumber(name + " REF ANGLE", state.angle.getRadians());
        setReferenceAngle(state.angle.getRadians());
    }

    @Override
    public void setDriveVoltage(double voltage) {
        driveMotor.set(voltage / MK4iConstants.REFERENCE_VOLTAGE);
    }
}
