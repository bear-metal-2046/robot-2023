package org.tahomarobotics.robot.util;

import edu.wpi.first.wpilibj.Filesystem;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Date;
import java.util.UUID;

public class SystemLogger {

    private static final UUID RUN_INSTANCE_UUID = UUID.randomUUID();

    private static final File HOME_DIR = Filesystem.getOperatingDirectory();

    public static void logRobotConstruction() {
        logMarker("robot startup", true);
    }

    public static void logRobotInit() {
        logMarker("robot init");
    }

    public static void logTeleopInit() {
        logMarker("teleop init");
    }

    public static void logAutoInit() {
        logMarker("auto init");
    }

    public static void logDisabledInit() {
        logMarker("disabled init");
    }

    public static void logTestInit() {
        logMarker("test init");
    }

    public static void logThrowableCrash(Throwable throwable) {
        logMarker("Exception", throwable, false);
    }

    private static void logMarker(String mark) {
        logMarker(mark, null, false);
    }
    private static void logMarker(String mark, boolean first) {
        logMarker(mark, null, first);
    }

    private static void logMarker(String mark, Throwable nullableException, boolean first) {

        try (PrintWriter writer = new PrintWriter(new FileWriter(new File(HOME_DIR, "crash_tracking.txt"), true))) {
            if (first) writer.println();

            writer.print(RUN_INSTANCE_UUID);
            writer.print(", ");
            writer.print(mark);
            writer.print(", ");
            writer.print(new Date());

            if (nullableException != null) {
                writer.print(", ");
                nullableException.printStackTrace(writer);
            }

            writer.println();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
