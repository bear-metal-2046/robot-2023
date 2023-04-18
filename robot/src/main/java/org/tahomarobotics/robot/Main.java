package org.tahomarobotics.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import org.tahomarobotics.robot.util.ChartData;
import org.tahomarobotics.robot.util.DebugChartData;
import org.tahomarobotics.robot.util.SystemLogger;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.StandardCopyOption;

public class Main {

    private static void initializeSerializeWorkaround() {
        ChartData chartData = new ChartData(
                "Workaround", " Time ( Sec", "Velocity (mps)",
                new String[]{"expected-vel", "actual-vel", "voltage", "expected-pos", "actual-pos"});

        for (double time = 0; time < 2.0; time += 0.02) {
            chartData.addData(new double[] { time, time, time, time, time, time});
        }

        chartData.serialize();
    }

    public static void main(String... args) {

        installWatchdog();

        SystemLogger.logRobotConstruction();

        try {

            if (DebugChartData.isEnabled()) {
                initializeSerializeWorkaround();
            }

            RobotBase.startRobot(() -> {
                try {
                    var robot = new Robot();
                    return robot;
                } catch (Throwable e) {
                    SystemLogger.logThrowableCrash(e);
                    throw new RuntimeException("Robot startup", e);
                }
            });

        } catch (Throwable t) {
            SystemLogger.logThrowableCrash(t);
        }
    }

    private static void installWatchdog() {
        String script = "monitor.sh";
        File deployScript = new File(Filesystem.getDeployDirectory(), script);
        File copiedScript = new File(Filesystem.getOperatingDirectory(), script);
        if (deployScript.exists()) {
            try {
                Files.copy(deployScript.toPath(), copiedScript.toPath(), StandardCopyOption.REPLACE_EXISTING);
                copiedScript.setExecutable(true);
                if (copiedScript.exists() && copiedScript.canExecute() && copiedScript.canRead()) {
                   Runtime.getRuntime().exec(copiedScript.getAbsolutePath());
                }
            } catch (IOException e) {
                e.printStackTrace(System.err);
                SystemLogger.logThrowableCrash(e);
            }
        } else {
            System.err.println("Failed to find executable script for monitoring JVM");
        }
    }
}
