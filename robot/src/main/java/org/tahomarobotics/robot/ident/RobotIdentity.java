package org.tahomarobotics.robot.ident;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.IOException;
import java.net.NetworkInterface;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Enumeration;
import java.util.List;

/**
 * Robot Identity Class
 * This class is utilized to sort out what robot we're working with, and assists with configuration on a robot by robot basis.
 */
public class RobotIdentity {
    /**
     * Initialize this classes logger.
     */
    private final Logger log = LoggerFactory.getLogger(RobotIdentity.class);

    private static RobotID robotID;

    private static final RobotIdentity INSTANCE = new RobotIdentity();

    public static RobotIdentity getInstance() {
        return INSTANCE;
    }

    private RobotIdentity() {
        for (byte[] actual : getRobotAddresses()) {
            for(RobotID ident : RobotID.values()) {
                if(Arrays.equals(ident.getMAC(), actual)) {
                    robotID = ident;
                    break;
                }
            }
            if (robotID != null) break;
        }

        if (robotID == null) {
            robotID = RobotID.COMPETITION;
            log.error("Failed to find a valid RobotID for the current robot, reverting to default.");
        }
        log.info("Robot Identity Determined as, " + robotID);

    }

    private List<byte[]> getRobotAddresses() {
        List<byte[]> macAddresses = new ArrayList<>();

        try {
            Enumeration<NetworkInterface> networkInterfaces = NetworkInterface.getNetworkInterfaces();

            NetworkInterface networkInterface;
            while (networkInterfaces.hasMoreElements()) {
                networkInterface = networkInterfaces.nextElement();

                byte[] address = networkInterface.getHardwareAddress();
                if (address == null) {
                    continue;
                }

                macAddresses.add(address);
                log.info("MAC address: " + macToString(address));
            }
        } catch (IOException e) {
            log.error("Failed to find a MAC Address on the Robot...", e);
        }

        return macAddresses;
    }

    private String macToString(byte[] address) {
        StringBuilder builder = new StringBuilder();
        for (int i = 0; i < address.length; i++) {
            if (i != 0) {
                builder.append(':');
            }
            builder.append(String.format("%02X", address[i]));
        }
        return builder.toString();
    }

    public RobotID getRobotID() { return robotID; }
}
