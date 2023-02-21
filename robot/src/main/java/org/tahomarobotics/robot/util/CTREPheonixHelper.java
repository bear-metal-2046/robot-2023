package org.tahomarobotics.robot.util;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import org.slf4j.Logger;

import java.util.function.Supplier;

public class CTREPheonixHelper extends BaseHelper {

    public static boolean needsConfiguring(Logger logger, CANCoder encoder, CANCoderConfiguration config) {
        return (
            isDifferent(logger, "Range",        encoder::configGetAbsoluteSensorRange,          config.absoluteSensorRange) ||
            isDifferent(logger, "Direction",    encoder::configGetSensorDirection,              config.sensorDirection) ||
            isDifferent(logger, "InitStrategy", encoder::configGetSensorInitializationStrategy, config.initializationStrategy) ||
            isDifferent(logger, "MagnetOffset", encoder::configGetMagnetOffset,                 config.magnetOffsetDegrees, 0.1) ||
            isDifferent(logger, "Coefficient",  encoder::configGetFeedbackCoefficient,          config.sensorCoefficient) ||
            isDifferent(logger, "Units",        encoder::configGetFeedbackUnitString,           config.unitString) ||
            isDifferent(logger, "TimeBase",     encoder::configGetFeedbackTimeBase,             config.sensorTimeBase)
        );
    }

    public static void configure(Logger logger, CANCoder encoder, CANCoderConfiguration config) {
        checkCtreError(logger, "configFactoryDefault", encoder::configFactoryDefault);
        checkCtreError(logger, "configAbsoluteSensorRange", () -> encoder.configAbsoluteSensorRange(config.absoluteSensorRange));
        checkCtreError(logger, "configSensorDirection", () -> encoder.configSensorDirection(config.sensorDirection));
        checkCtreError(logger, "configSensorInitializationStrategy", () -> encoder.configSensorInitializationStrategy(config.initializationStrategy));
        checkCtreError(logger, "configFeedbackCoefficient", () -> encoder.configFeedbackCoefficient(config.sensorCoefficient, config.unitString, config.sensorTimeBase));
    }

    public static void checkThenConfigure(String name, Logger logger, CANCoder encoder, CANCoderConfiguration config) {
        if (CTREPheonixHelper.needsConfiguring(logger, encoder, config)) {
            logger.warn("Configuring " + name);
            CTREPheonixHelper.configure(logger, encoder, config);
        }
    }

    /**
     * Check the return code of the function and logs and error if applicable
     *
     * @param logger - logger provided by subsystem
     * @param functionName - function being called
     * @param func - function to be called
     */
    private static void checkCtreError(Logger logger, String functionName, Supplier<ErrorCode> func)  {
        ErrorCode error = func.get();
        if (error != ErrorCode.OK) {
            logger.error("Failed calling " + functionName + " " + error);
        }
    }

}

