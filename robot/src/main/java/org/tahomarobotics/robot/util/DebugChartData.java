package org.tahomarobotics.robot.util;

import edu.wpi.first.networktables.NetworkTableInstance;

public class DebugChartData {



    private static boolean _isEnabled() {
        return NetworkTableInstance.getDefault().getTable("SmartDashboard")
                .getEntry("Bear Scope Enabled")
                .getBoolean(false);
    }

    private static final boolean enabled = _isEnabled();

    public static boolean isEnabled() {
        return enabled;
    }
}
