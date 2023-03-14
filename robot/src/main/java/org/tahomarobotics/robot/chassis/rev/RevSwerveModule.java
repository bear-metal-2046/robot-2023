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
package org.tahomarobotics.robot.chassis.rev;

import com.revrobotics.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.chassis.SwerveModuleIF;
import org.tahomarobotics.robot.util.SparkMaxConfig;
import org.tahomarobotics.robot.util.SparkMaxHelper;

/**
 * SwerveModule Class
 * Handles setup and various utility methods for Swerve Modules.
 *
 */
public class RevSwerveModule implements SwerveModuleIF {

    private static final Logger logger = LoggerFactory.getLogger(RevSwerveModule.class);

    private final String name;

    private final CANSparkMax driveMotor;
    private final CANSparkMax steerMotor;
    private final AbsoluteEncoder steerABSEncoder;

    private final PIDController drivePIDController = new PIDController(0, 0, 0);
    private SparkMaxPIDController steerPIDController;
    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0, RevChassisConstants.kV_DRIVE);
    private SwerveModuleState state = new SwerveModuleState();
    private SwerveModulePosition position = new SwerveModulePosition();

    private double lastTime = RobotController.getFPGATime() * 1e-6;

    private final Translation2d positionOffset;

    private double angularOffset;

    public RevSwerveModule(String name, RobotMap.SwerveModulePorts ports, Translation2d positionOffset, double angularOffset) {
        this.name = name;
        this.positionOffset = positionOffset;
        this.angularOffset = angularOffset;

        // configure drive motor
        driveMotor = new CANSparkMax(ports.drive(), CANSparkMaxLowLevel.MotorType.kBrushless);
        SparkMaxConfig driveConfig = RevChassisConstants.createDriveConfig(ports.drive());

        SparkMaxHelper.checkThenConfigure(name, logger, driveConfig, driveMotor, driveMotor.getEncoder());
        drivePIDController.setPID(driveConfig.kP, driveConfig.kI, driveConfig.kD);

        driveMotor.getEncoder().setPosition(0.0);

        // configure steer motor
        steerMotor = new CANSparkMax(ports.steer(), CANSparkMaxLowLevel.MotorType.kBrushless);
        steerABSEncoder = steerMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        steerPIDController = steerMotor.getPIDController();
        SparkMaxConfig steerConfig = RevChassisConstants.createSteerConfig(ports.steer(), angularOffset);

        SparkMaxHelper.checkThenConfigure(name + " Steer", logger, steerConfig, steerMotor, steerABSEncoder, steerPIDController);
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
    public Translation2d getPositionOffset() {
        return positionOffset;
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
        return RobotBase.isReal() ? new SwerveModuleState(getVelocity(), new Rotation2d(getAbsoluteAngle())) : state;
    }

    /**
     * Returns the current position of the module.
     * @return The current position of the module.
     */
    @Override
    public SwerveModulePosition getPosition() {
        // This code is speculative as the documentation and examples on is non-existent
        return RobotBase.isReal() ? new SwerveModulePosition(getDrivePos(), new Rotation2d(getAbsoluteAngle())) : position;
    }

    @Override
    public double getDriveVelocity() {
        SwerveModuleState _state = SwerveModuleState.optimize(getState(), new Rotation2d(getAbsoluteAngle()));
        return _state.speedMetersPerSecond;
    }

    /**
     * Sets the desired state for the module.
     * @param desiredState Desired state with speed and angle.
     */
    @Override
    public void setDesiredState(SwerveModuleState desiredState) {
        double steerAngle = getAbsoluteAngle();

        state = SwerveModuleState.optimize(desiredState, new Rotation2d(steerAngle));

        double time = RobotController.getFPGATime() * 1e-6;
        position.angle = state.angle;
        position.distanceMeters += state.speedMetersPerSecond * (time - lastTime);
        lastTime = time;

        // Calculate the drive output from the drive PID controller.
        double driveOutput = drivePIDController.calculate(getVelocity(), state.speedMetersPerSecond);
        driveOutput += driveFeedforward.calculate(state.speedMetersPerSecond);

        setDriveVoltage(driveOutput);
        steerPIDController.setReference(state.angle.getRadians(), CANSparkMax.ControlType.kPosition);
    }

    public void setDriveVoltage(double voltage) {
        driveMotor.setVoltage(voltage);
    }

    //TODO Alignment.
    @Override
    public double finalizeCalibration() {
        angularOffset = getAbsoluteAngle();
        applyAngularOffset(angularOffset);
        steerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        return angularOffset;
    }

    @Override
    public void initializeCalibration() {
        applyAngularOffset(0);
        steerMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    @Override
    public void cancelCalibration() {
        applyAngularOffset(angularOffset);
    }

    private void applyAngularOffset(double offset) {
        var rtncode = steerABSEncoder.setZeroOffset(offset);
        if (rtncode != REVLibError.kOk) {
            logger.error("Failed to apply angular offset to " + offset);
        }
    }

    @Override
    public void displayPosition() {
        SmartDashboard.putNumber(name + " angle", getAbsoluteAngle());
    }
}
