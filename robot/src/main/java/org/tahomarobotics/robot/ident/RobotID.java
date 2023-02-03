package org.tahomarobotics.robot.ident;

/**
 * RobotID that stores specific robot identifications via MAC Addresses.
 */
public enum RobotID {

    /**
     * ALPHA Bot used during build-season. Configuration values will be unstable.
     */
    ALPHA {
        @Override
        public byte[] getMAC() {
            //ALPHA Bot Addr not defined at this time.
            return new byte[] {0x00, (byte) 0x80, 0x2F, 0x28, 0x50, 0x53};
        }
    },

    /**
     * Prototype Bot used during build-season. Configuration values may be unstable.
     */
    PROTOTYPE {
        @Override
        public byte[] getMAC() {
            return new byte[]{0x00, (byte) 0x80, 0x2F, 0x32, (byte) 0xFD, 0x29};
        }
    },

    /**
     * Competition Robot, all configuration values should be competition ready.
     */
    COMPETITION {
        @Override
        public byte[] getMAC() {
            return new byte[]{0x00, (byte) 0x80, 0x2F, 0x32, (byte) 0xFC, (byte) 0xD7};
        }
    },

    UNDEFINED {
        @Override
        public byte[] getMAC() {
            return null;
        }
    };

    /**
     * Fetches MAC address for specific robot.
     * @return null by default, defined by enum values.
     */
    public abstract byte[] getMAC();
}
