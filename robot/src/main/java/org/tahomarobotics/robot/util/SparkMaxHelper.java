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
package org.tahomarobotics.robot.util;

import com.revrobotics.*;
import org.slf4j.Logger;

import java.util.function.Supplier;

public class SparkMaxHelper extends BaseHelper{

    /**
     * Compares the current configuration of the motor controller with the provided configuration.
     *
     * @param logger - logger provided by subsystem
     * @param cfg - motor config file
     * @param motor - SparkMax controller
     * @return true if different
     */
    public static boolean needsConfiguring(Logger logger, SparkMaxConfig cfg, CANSparkMax motor) {
        return needsConfiguring(logger, cfg, motor, null, null);
    }

    /**
     * Compares the current configuration of the motor controller and feedback sensor with the provided configuration.
     *
     * @param logger - logger provided by subsystem
     * @param cfg - motor config file
     * @param motor - SparkMax controller
     * @param feedbackSensor - either absolute or relative encoder
     * @return true if different
     */
    public static boolean needsConfiguring(Logger logger, SparkMaxConfig cfg, CANSparkMax motor, MotorFeedbackSensor feedbackSensor) {
        return needsConfiguring(logger, cfg, motor, feedbackSensor, null);
    }

    /**
     * Compares the current configuration of the motor controller, feedback sensor, and pid controller with the provided
     * configuration.
     *
     * @param logger - logger provided by subsystem
     * @param cfg - motor config file
     * @param motor - SparkMax controller
     * @param feedbackSensor - either absolute or relative encoder
     * @param pidController - pid controller
     * @return true if different
     */
    public static boolean needsConfiguring(Logger logger, SparkMaxConfig cfg, CANSparkMax motor, MotorFeedbackSensor feedbackSensor, SparkMaxPIDController pidController) {

        return (
             isDifferent(logger, "idleBrake", motor::getIdleMode, cfg.idleBrake) ||
             isDifferent(logger, "Inverted", motor::getInverted, cfg.motorInverted) ||
             isDifferent(logger, "compensationNominal", motor::getVoltageCompensationNominalVoltage, cfg.compensationNominal) ||
             needsConfiguring(logger, cfg, feedbackSensor) ||
             needsConfiguring(logger, cfg, pidController)
        );
    }

    /**
     * Calls the appropriate encoder
     */
    private static boolean needsConfiguring(Logger logger, SparkMaxConfig cfg, MotorFeedbackSensor feedbackSensor) {
        if (feedbackSensor != null) {
            if (feedbackSensor instanceof AbsoluteEncoder) {
                return needsConfiguring(logger, cfg, (AbsoluteEncoder)feedbackSensor);

            } else if (feedbackSensor instanceof RelativeEncoder) {
                return needsConfiguring(logger, cfg, (RelativeEncoder)feedbackSensor);
            }
        }
        return false;
    }

    /**
     * Checks the AbsoluteEncoder
     */
    private static boolean needsConfiguring(Logger logger, SparkMaxConfig cfg, AbsoluteEncoder encoder) {
        return (
             isDifferent(logger, "encoderInverted", encoder::getInverted, cfg.encoderInverted) ||
             isDifferent(logger, "positionConversionFactor", encoder::getPositionConversionFactor, cfg.positionConversionFactor) ||
             isDifferent(logger, "velocityConversionFactor", encoder::getVelocityConversionFactor, cfg.velocityConversionFactor) ||
             isDifferent(logger, "encoderOffset", encoder::getZeroOffset, cfg.encoderOffset)
          );
    }

    /**
     * Checks the RelativeEncoder
     */
    private static boolean needsConfiguring(Logger logger, SparkMaxConfig cfg, RelativeEncoder encoder) {
        return (
             isDifferent(logger, "encoderInverted", encoder::getInverted, cfg.encoderInverted) ||
             isDifferent(logger, "positionConversionFactor", encoder::getPositionConversionFactor, cfg.positionConversionFactor) ||
             isDifferent(logger, "velocityConversionFactor", encoder::getVelocityConversionFactor, cfg.velocityConversionFactor)
        );
    }

    /**
     * Checks the pid controller
     */
    private static boolean needsConfiguring(Logger logger, SparkMaxConfig cfg, SparkMaxPIDController pidController) {
        if (pidController == null) {
            return false;
        }
        return (
             isDifferent(logger, "kP", pidController::getP, cfg.kP) ||
             isDifferent(logger, "kI", pidController::getI, cfg.kI) ||
             isDifferent(logger, "kD", pidController::getD, cfg.kD) ||
             isDifferent(logger, "kFF", pidController::getFF, cfg.kFF) ||
             isDifferent(logger, "wrapMin", pidController::getPositionPIDWrappingMinInput, cfg.wrapMin) ||
             isDifferent(logger, "wrapMax", pidController::getPositionPIDWrappingMaxInput, cfg.wrapMax) ||
             isDifferent(logger, "wrapEnabled", pidController::getPositionPIDWrappingEnabled, cfg.wrapEnabled )
        );
    }

    /**
     * Configures the provided SparkMax controller with the provided configuration after resetting defaults.
     * It finishes with burning to flash to ensure it starts with that configuration the next time.
     * This checks for errors reported by the SparkMaxController.
     *
     * @param logger - logger provided by subsystem
     * @param cfg - motor config file
     * @param motor - SparkMax controller
     */
    public static void configure(Logger logger, SparkMaxConfig cfg, CANSparkMax motor) {
        configure(logger, cfg, motor, null, null);
    }

    /**
     * Configures the provided SparkMax controller and encoder with the provided configuration after resetting defaults.
     * It finishes with burning to flash to ensure it starts with that configuration the next time.
     * This checks for errors reported by the SparkMaxController.
     *
     * @param logger - logger provided by subsystem
     * @param cfg - motor config file
     * @param motor - SparkMax controller
     * @param feedbackSensor - either absolute or relative encoder
     */
    public static void configure(Logger logger, SparkMaxConfig cfg, CANSparkMax motor, MotorFeedbackSensor feedbackSensor) {
        configure(logger, cfg, motor, feedbackSensor, null);
    }

    /**
     * Configures the provided SparkMax controller, encoder and pid controller with the provided configuration after resetting defaults.
     * It finishes with burning to flash to ensure it starts with that configuration the next time.
     * This checks for errors reported by the SparkMaxController.
     *
     * @param logger - logger provided by subsystem
     * @param cfg - motor config file
     * @param motor - SparkMax controller
     * @param feedbackSensor - either absolute or relative encoder
     * @param pidController - pid controller
     */
    public static void configure(Logger logger, SparkMaxConfig cfg, CANSparkMax motor, MotorFeedbackSensor feedbackSensor, SparkMaxPIDController pidController) {
        configure(logger, cfg, motor, feedbackSensor, pidController, null);
    }

        /**
         * Configures the provided SparkMax controller, encoder and pid controller with the provided configuration after resetting defaults.
         * It finishes with burning to flash to ensure it starts with that configuration the next time.
         * This checks for errors reported by the SparkMaxController.
         *
         * @param logger - logger provided by subsystem
         * @param cfg - motor config file
         * @param motor - SparkMax controller
         * @param feedbackSensor - either absolute or relative encoder
         * @param pidController - pid controller
         * @param follower - motor that will follow
         */
    public static void configure(Logger logger, SparkMaxConfig cfg, CANSparkMax motor, MotorFeedbackSensor feedbackSensor, SparkMaxPIDController pidController, CANSparkMax follower) {

        checkRevError(logger, "restoreFactoryDefaults", motor::restoreFactoryDefaults);

        motor.setInverted(cfg.motorInverted);
        checkRevError(logger,"setIdleMode", () -> motor.setIdleMode(cfg.idleBrake));
        checkRevError(logger,"enableVoltageCompensation", () -> motor.enableVoltageCompensation(cfg.compensationNominal));

        if (feedbackSensor!= null) {
            if (pidController != null) {
                checkRevError(logger,"setFeedbackSesnor", () -> pidController.setFeedbackDevice(feedbackSensor));
            }
            if (feedbackSensor instanceof AbsoluteEncoder) {
                configure(logger, cfg, (AbsoluteEncoder)feedbackSensor);

            } else if (feedbackSensor instanceof RelativeEncoder) {
                configure(logger, cfg, (RelativeEncoder)feedbackSensor);
            }
        }

        if (pidController != null) {
            configure(logger, cfg, pidController);
        }

        if (follower != null) {
            checkRevError(logger, "follower: restoreFactoryDefaults", follower::restoreFactoryDefaults);
            checkRevError(logger, "follower: follow", () ->follower.follow(motor));
            checkRevError(logger,"follower: burnFlash", follower::burnFlash);
        }

        checkRevError(logger,"burnFlash", motor::burnFlash);
    }

    /**
     * Configures an AbsoluteEncoder
     *
     * @param logger - logger provided by subsystem
     * @param cfg - motor config file
     * @param encoder - absolute encoder
     */
    private static void configure(Logger logger, SparkMaxConfig cfg, AbsoluteEncoder encoder) {
        checkRevError(logger,"posConv", () -> encoder.setPositionConversionFactor(cfg.positionConversionFactor));
        checkRevError(logger,"velConv", () -> encoder.setVelocityConversionFactor(cfg.velocityConversionFactor));
        checkRevError(logger,"encInv", () -> encoder.setInverted(cfg.encoderInverted));
        checkRevError(logger,"offset->"+cfg.encoderOffset, () -> encoder.setZeroOffset(cfg.encoderOffset));
    }

    /**
     * Configures a RelativeEncoder
     *
     * @param logger - logger provided by subsystem
     * @param cfg - motor config file
     * @param encoder - relative encoder
     */
    private static void configure(Logger logger, SparkMaxConfig cfg, RelativeEncoder encoder) {
        checkRevError(logger,"posConv", () -> encoder.setPositionConversionFactor(cfg.positionConversionFactor));
        checkRevError(logger,"velConv", () -> encoder.setVelocityConversionFactor(cfg.velocityConversionFactor));
    }

    /**
     * Configure a pid controller
     *
     * @param logger - logger provided by subsystem
     * @param cfg - motor config file
     * @param pidController - pid controller
     */
    private static void configure(Logger logger, SparkMaxConfig cfg, SparkMaxPIDController pidController) {
        checkRevError(logger,"setPositionPIDWrappingEnabled", () -> pidController.setPositionPIDWrappingEnabled(cfg.wrapEnabled));
        checkRevError(logger,"setPositionPIDWrappingMinInput", () -> pidController.setPositionPIDWrappingMinInput(cfg.wrapMin));
        checkRevError(logger,"setPositionPIDWrappingMaxInput", () -> pidController.setPositionPIDWrappingMaxInput(cfg.wrapMax));
        checkRevError(logger,"setP", () -> pidController.setP(cfg.kP));
        checkRevError(logger,"setI", () -> pidController.setI(cfg.kI));
        checkRevError(logger,"setD", () -> pidController.setD(cfg.kD));
        checkRevError(logger,"setFF", () -> pidController.setFF(cfg.kFF));
    }

    /**
     * Check the return code of the function and logs and error if applicable
     *
     * @param logger - logger provided by subsystem
     * @param functionName - function being called
     * @param func - function to be called
     */
    private static void checkRevError(Logger logger, String functionName, Supplier<REVLibError> func)  {
        REVLibError error = func.get();
        if (error != REVLibError.kOk) {
            logger.error("Failed calling " + functionName + " " + error);
        }
    }

    public static void checkThenConfigure(String name, Logger logger, SparkMaxConfig cfg, CANSparkMax motor) {
        if (needsConfiguring(logger, cfg, motor)) {
            logger.warn("Configuring " + name);
            configure(logger, cfg, motor);
        }
    }

    public static void checkThenConfigure(String name, Logger logger, SparkMaxConfig cfg, CANSparkMax motor, CANSparkMax follower) {
        if (needsConfiguring(logger, cfg, motor)) {
            logger.warn("Configuring " + name);
            configure(logger, cfg, motor, null, null, follower);
        }
    }
    public static void checkThenConfigure(String name, Logger logger, SparkMaxConfig cfg, CANSparkMax motor, RelativeEncoder encoder) {
        if (needsConfiguring(logger, cfg, motor, encoder)) {
            logger.warn("Configuring " + name);
            configure(logger, cfg, motor, encoder, null, null);
        }
    }
}
