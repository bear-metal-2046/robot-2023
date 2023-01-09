package org.tahomarobotics.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * New Logging Management Class
 * This class dumps information to log files & straight back to DriverStation.
 * Reference the class directly.
 */
public class LoggerManager {
    private static final Logger logger = LoggerFactory.getLogger(LoggerManager.class);

    public static void log(String msg) {
        DriverStation.reportWarning(msg, false);
        logger.info(msg);
    }

    public static void warn(String msg) {
        DriverStation.reportWarning(" -=-=- WARNING -=-=- " + msg + " -=-=- WARNING -=-=-",false);
        logger.warn(msg);
    }

    public static void error(String msg) {
        DriverStation.reportError(msg, true);
        logger.error(msg);
    }
}
