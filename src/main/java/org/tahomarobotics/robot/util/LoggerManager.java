package org.tahomarobotics.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class LoggerManager {
    private static Logger logger = LoggerFactory.getLogger(LoggerManager.class);

    public static void log(String msg) {
        DriverStation.reportWarning(msg, false);
        logger.info(msg);
    }

    public static void warn(String msg) {
        DriverStation.reportWarning(" -=-=- WARNING -=-=- " + msg + " -=-=- WARNING -=-=-",false);
        logger.warn(msg);
    }

    public static void error(String msg) {
        DriverStation.reportError(msg, false);
        logger.error(msg);
    }
}
