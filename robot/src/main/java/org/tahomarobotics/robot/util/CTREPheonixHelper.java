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

