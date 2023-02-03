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

import com.revrobotics.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.chassis.config.REVMaxConstants;

/**
 * SwerveModule Class
 * Handles setup and various utility methods for Swerve Modules.
 * @implNote Due to VEX being a non-option for swerve this year, our swerve modules will not be using TalonFX's or CTRE products.
 *
 */
public class MAXSwerveModule implements SwerveModuleIF {

    private static final Logger logger = LoggerFactory.getLogger(MAXSwerveModule.class);

    public record SwerveConfiguration(String name, RobotMap.SwerveModulePorts ports, double referenceAngle) {
    }

    private final String name;

    private final CANSparkMax driveMotor;
    private final CANSparkMax steerMotor;
    private final AbsoluteEncoder steerABSEncoder;

    private final PIDController drivePIDController = new PIDController(0, 0, 0);
    private SparkMaxPIDController steerPIDController;
    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0, REVMaxConstants.kV_DRIVE);
    private SwerveModuleState state = new SwerveModuleState();

    private final SparkMaxConfig steerConfig;

    private final SparkMaxConfig driveConfig;



    public MAXSwerveModule(SwerveConfiguration configuration) {
        this(configuration.name, configuration.ports, configuration.referenceAngle);
    }

    public MAXSwerveModule(String name, RobotMap.SwerveModulePorts ports, double referenceAngle) {
        this.name = name;
        driveMotor = new CANSparkMax(ports.drive(), CANSparkMaxLowLevel.MotorType.kBrushless);
        steerMotor = new CANSparkMax(ports.steer(), CANSparkMaxLowLevel.MotorType.kBrushless);
        steerABSEncoder = steerMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        steerPIDController = steerMotor.getPIDController();
        driveConfig = setDriveConfig(ports.drive());
        steerConfig = setSteerConfig(ports.steer(), referenceAngle);
        if (SparkMaxHelper.needsConfiguring(driveConfig, driveMotor, driveMotor.getEncoder())) {
            logger.error("Configured DRIVE motors");
            SparkMaxHelper.configure(logger, driveConfig, driveMotor, driveMotor.getEncoder());
        }
        if (SparkMaxHelper.needsConfiguring(steerConfig, steerMotor, steerABSEncoder, steerPIDController)) {
            logger.error("Configured STEER motors");
            SparkMaxHelper.configure(logger, steerConfig, steerMotor, steerABSEncoder, steerPIDController);
        }
    }
    private SparkMaxConfig setDriveConfig(int id) {
        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.canId = id;
        cfg.currentLimit = (int) REVMaxConstants.DRIVE_CURRENT_LIMIT;
        cfg.positionConversionFactor = REVMaxConstants.DRIVE_ENCODER_POSITION_FACTOR;
        cfg.velocityConversionFact = REVMaxConstants.DRIVE_ENCODER_VELOCITY_FACTOR;
        cfg.kP = 0.04;
        cfg.kFF = 1 / REVMaxConstants.DRIVE_WHEEL_FREE_SPEED_RPS;
        return cfg;
    }
    private SparkMaxConfig setSteerConfig(int id, double offset) {
        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.canId = id;
        cfg.currentLimit = (int) REVMaxConstants.STEER_CURRENT_LIMIT;
        cfg.positionConversionFactor = REVMaxConstants.STEER_ENCODER_POSITION_FACTOR;
        cfg.velocityConversionFact = REVMaxConstants.STEER_ENCODER_VELOCITY_FACTOR;
        cfg.encoderInverted = true;
        cfg.encoderOffset = offset;
        cfg.kP = 0.75;
        cfg.wrapEnabled = true;
        return cfg;
    }

    @Override
    public double getAbsoluteAngle() {
        double angle = steerABSEncoder.getPosition();
        angle %= 2.0 * Math.PI;
        if (angle < 0.0) {
            angle += 2.0 * Math.PI;
        }

        return angle;
    }

    @Override
    public double getVelocity() {
        return driveMotor.getEncoder().getVelocity();
    }

    public double getDrivePos() {
        return driveMotor.getEncoder().getPosition();
    }

    /**
     * Returns the current state of the module.
     * @return The current state of the module.
     */
    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), new Rotation2d(getAbsoluteAngle()));
    }

    /**
     * Returns the current position of the module.
     * @return The current position of the module.
     */
    @Override
    public SwerveModulePosition getPosition() {
        // This code is speculative as the documentation and examples on is non-existent
        return new SwerveModulePosition(getDrivePos(), new Rotation2d(getAbsoluteAngle()));
    }

    /**
     * Sets the desired state for the module.
     * @param desiredState Desired state with speed and angle.
     */
    @Override
    public void setDesiredState(SwerveModuleState desiredState) {
        double steerAngle = getAbsoluteAngle();

        state = SwerveModuleState.optimize(desiredState, new Rotation2d(steerAngle));

        // Calculate the drive output from the drive PID controller.
        final double driveOutput = drivePIDController.calculate(getVelocity(), state.speedMetersPerSecond);
        final double driveFeedforward = this.driveFeedforward.calculate(state.speedMetersPerSecond);

        setDriveVoltage(driveOutput + driveFeedforward);
        SmartDashboard.putNumber(name + " REF ANGLE", state.angle.getRadians());
        steerPIDController.setReference(state.angle.getRadians(), CANSparkMax.ControlType.kPosition);
    }

    public void setDriveVoltage(double voltage) {
        driveMotor.set(voltage / REVMaxConstants.REFERENCE_VOLTAGE);
    }

    //TODO Alignment.
    @Override
    public void align() {

    }

    @Override
    public void zeroOffset() {

    }

    @Override
    public void updateOffset() {

    }

    @Override
    public void displayPosition() {

    }
}
