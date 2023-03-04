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
package org.tahomarobotics.robot.climb;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.SubsystemIF;
import org.tahomarobotics.robot.motion.MotionState;
import org.tahomarobotics.robot.util.SparkMaxConfig;
import org.tahomarobotics.robot.util.SparkMaxHelper;

public class Paw extends SubsystemBase implements SubsystemIF {
    private static final Logger logger = LoggerFactory.getLogger(Paw.class);
    private int x;
    private static final Paw LEFT_INSTANCE =
            new Paw("Left Paw", ClimbConstants.createPawConfig(RobotMap.LEFT_PAW, true), 0);
    private static final Paw RIGHT_INSTANCE =
            new Paw("Right Paw", ClimbConstants.createPawConfig(RobotMap.RIGHT_PAW, false), 1);
    public static Paw getLeftInstance() { return LEFT_INSTANCE; }
    public static Paw getRightInstance() { return RIGHT_INSTANCE; }
    private final String name;
    private final CANSparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkMaxPIDController pidController;
    private PawShuffleboard shuffleboard;
    private Paw(String name, SparkMaxConfig motorConfig, int x) {
        this.x = x;
        this.name = name;

        motor = new CANSparkMax(motorConfig.canId, CANSparkMaxLowLevel.MotorType.kBrushless);

        pidController = motor.getPIDController();
        encoder = motor.getEncoder();

        SparkMaxHelper.checkThenConfigure(name, logger, motorConfig, motor, encoder, pidController);
    }

    @Override
    public Paw initialize() {
        shuffleboard = new PawShuffleboard(name, x, this);

        Commands.waitUntil(RobotState::isEnabled)
                .andThen(new ZeroPawCommand(this))
                .andThen(new PawCommand(this, Units.degreesToRadians(20), 2))
                .ignoringDisable(true).schedule();

        return this;
    }

    @Override
    public void periodic() {
        shuffleboard.update();
        //SmartDashboard.putNumber(name + "Pos", getPos());
    }

    public void setGoal(MotionState setpoint) {

        double feedForwardVoltage = ClimbConstants.kV * setpoint.velocity + ClimbConstants.kA * setpoint.acceleration;

        pidController.setReference(setpoint.position, CANSparkMax.ControlType.kPosition, 0, feedForwardVoltage);
    }

    public void setPower(double power) {
        motor.set(power);
    }

    public void zeroEncoder() {
        encoder.setPosition(0d);
        logger.info(name + " encoder got zeroed");
    }

    public double getPos() {
        return motor.getEncoder().getPosition();
    }
    public double getVelocity() {
        return encoder.getVelocity();
    }

    public void setZeroCurrentLimit(boolean isZeroLimit) {
        motor.setSmartCurrentLimit(isZeroLimit ? ClimbConstants.ZERO_CURRENT_LIMIT : ClimbConstants.FULL_CURRENT_LIMIT);
    }

    public String getName() {
        return name;
    }
}