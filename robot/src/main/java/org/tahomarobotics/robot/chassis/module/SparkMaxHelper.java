package org.tahomarobotics.robot.chassis.module;

import com.revrobotics.*;
import org.slf4j.Logger;

import java.util.function.Supplier;

public class SparkMaxHelper {

    private static final double EPSILON = 0.000001d;

    private static boolean isEqual(double a, double b, double eps) {
        return Math.abs(a-b) < eps;
    }

    private static boolean isEqual(double a, double b) {
        return isEqual(a, b, EPSILON);
    }

    public static boolean needsConfiguring(SparkMaxConfig cfg, CANSparkMax motor) {
        return needsConfiguring(cfg, motor, null, null);
    }
    public static boolean needsConfiguring(SparkMaxConfig cfg, CANSparkMax motor, MotorFeedbackSensor feedbackSensor) {
        return needsConfiguring(cfg, motor, feedbackSensor, null);
    }

    public static boolean needsConfiguring(SparkMaxConfig cfg, CANSparkMax motor, MotorFeedbackSensor feedbackSensor, SparkMaxPIDController pidController) {
        boolean needsConfiguring = false;

        needsConfiguring |= motor.getIdleMode() != cfg.idleBrake;
        needsConfiguring |= ! isEqual(motor.getVoltageCompensationNominalVoltage(), cfg.compensationNominal);

        if (feedbackSensor!= null) {
            if (feedbackSensor instanceof AbsoluteEncoder) {
                needsConfiguring |= needsConfiguring(cfg, (AbsoluteEncoder)feedbackSensor);

            } else if (feedbackSensor instanceof RelativeEncoder) {
                needsConfiguring |= needsConfiguring(cfg, (RelativeEncoder)feedbackSensor);
            }
        }

        if (pidController != null) {
            needsConfiguring |= needsConfiguring(cfg, pidController);
        }

        return needsConfiguring;
    }


    private static boolean needsConfiguring(SparkMaxConfig cfg, AbsoluteEncoder encoder) {
        return (
                encoder.getInverted() != cfg.encoderInverted ||
                ! isEqual(encoder.getZeroOffset(), cfg.encoderOffset) ||
                ! isEqual(encoder.getPositionConversionFactor(), cfg.positionConversionFactor) ||
                ! isEqual(encoder.getVelocityConversionFactor(), cfg.positionConversionFactor)
        );
    }
    private static boolean needsConfiguring(SparkMaxConfig cfg, RelativeEncoder encoder) {
        return (
                encoder.getInverted() != cfg.encoderInverted ||
                ! isEqual(encoder.getPositionConversionFactor(), cfg.positionConversionFactor) ||
                ! isEqual(encoder.getVelocityConversionFactor(), cfg.positionConversionFactor)
        );
    }
    private static boolean needsConfiguring(SparkMaxConfig cfg, SparkMaxPIDController pidController) {
        return (
                ! isEqual(pidController.getP() , cfg.kP) ||
                ! isEqual(pidController.getI() , cfg.kI) ||
                ! isEqual(pidController.getD() , cfg.kD) ||
                ! isEqual(pidController.getFF() , cfg.kFF) ||
                pidController.getPositionPIDWrappingEnabled() != cfg.wrapEnabled ||
                ! isEqual(pidController.getPositionPIDWrappingMinInput(), cfg.wrapMin) ||
                ! isEqual(pidController.getPositionPIDWrappingMaxInput(), cfg.wrapMax)
        );
    }

    public static void configure(Logger logger, SparkMaxConfig cfg, CANSparkMax motor) {
        configure(logger, cfg, motor, null, null);
    }
    public static void configure(Logger logger, SparkMaxConfig cfg, CANSparkMax motor, MotorFeedbackSensor feedbackSensor) {
        configure(logger, cfg, motor, feedbackSensor, null);
    }

    public static void configure(Logger logger, SparkMaxConfig cfg, CANSparkMax motor, MotorFeedbackSensor feedbackSensor, SparkMaxPIDController pidController) {

        checkRevError(logger, "restoreFactoryDefaults", () -> motor.restoreFactoryDefaults());

        motor.setInverted(cfg.motorInverted);
        checkRevError(logger,"setIdleMode", () -> motor.setIdleMode(cfg.idleBrake));

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

        checkRevError(logger,"burnFlash", () -> motor.burnFlash());
    }

    private static void configure(Logger logger, SparkMaxConfig cfg, AbsoluteEncoder encoder) {
        checkRevError(logger,"posConv", () -> encoder.setPositionConversionFactor(cfg.positionConversionFactor));
        checkRevError(logger,"velConv", () -> encoder.setVelocityConversionFactor(cfg.velocityConversionFact));
        checkRevError(logger,"encInv", () -> encoder.setInverted(cfg.encoderInverted));
        checkRevError(logger,"offset->"+cfg.encoderOffset, () -> encoder.setZeroOffset(cfg.encoderOffset));
    }

    private static void configure(Logger logger, SparkMaxConfig cfg, RelativeEncoder encoder) {
        checkRevError(logger,"posConv", () -> encoder.setPositionConversionFactor(cfg.positionConversionFactor));
        checkRevError(logger,"velConv", () -> encoder.setVelocityConversionFactor(cfg.velocityConversionFact));
    }

    private static void configure(Logger logger, SparkMaxConfig cfg, SparkMaxPIDController pidController) {
        checkRevError(logger,"setPositionPIDWrappingEnabled", () -> pidController.setPositionPIDWrappingEnabled(cfg.wrapEnabled));
        checkRevError(logger,"setPositionPIDWrappingMinInput", () -> pidController.setPositionPIDWrappingMinInput(cfg.wrapMin));
        checkRevError(logger,"setPositionPIDWrappingMaxInput", () -> pidController.setPositionPIDWrappingMaxInput(cfg.wrapMax));
        checkRevError(logger,"setP", () -> pidController.setP(cfg.kP));
        checkRevError(logger,"setI", () -> pidController.setI(cfg.kI));
        checkRevError(logger,"setD", () -> pidController.setD(cfg.kD));
        checkRevError(logger,"setFF", () -> pidController.setFF(cfg.kFF));
    }
    private static void checkRevError(Logger logger, String functionName, Supplier<REVLibError> func)  {
        REVLibError error = func.get();
        if (error != REVLibError.kOk) {
            logger.error("Failed calling " + functionName + " " + error);
        }
    }
}
