/**
 * Copyright 2022 Tahoma Robotics - http://tahomarobotics.org - Bear Metal 2046 FRC Team
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
package org.tahomarobotics.dashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.openfx.Dashboard;

/**
 * All Dashboard classes should use this enum to access the SmartDashboard.  Typical usage is:
 * <code>
 *     private final NetworkTable table = DashboardNetworkTable.INSTANCE.getSmartDashboard();
:w * </code>
 *     private final NetworkTable table = DashboardNetworkTable.INSTANCE.getSmartDashboard();
 */
public enum DashboardNetworkTable {
    INSTANCE;

    private final NetworkTableInstance netTable;
    private final NetworkTable table;
    private String host;

    DashboardNetworkTable () {
        netTable = NetworkTableInstance.getDefault();
        table = netTable.getTable("SmartDashboard");
        changeHost(Dashboard.ROBOT_IP);
    }

    public void changeHost(String host) {
        this.host = host;
        netTable.stopClient();
        netTable.startClient(host);
    }

    public NetworkTable getSmartDashboard() {
        return table;
    }

    public String getHost() {
        return host;
    }

}
