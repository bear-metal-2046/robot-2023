package org.tahomarobotics.robot;

import edu.wpi.first.wpilibj.RobotBase;
import org.tahomarobotics.robot.util.ChartData;
import org.tahomarobotics.robot.util.SystemLogger;

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

        SystemLogger.logRobotConstruction();

        try {

            initializeSerializeWorkaround();

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
}
