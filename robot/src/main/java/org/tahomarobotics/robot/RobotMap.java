package org.tahomarobotics.robot;

public final class RobotMap {
    public record SwerveModulePorts(int drive, int steer, int encoder) {}

    // Swerve Modules
    public static final SwerveModulePorts FRONT_LEFT_MOD = new SwerveModulePorts(1, 11, 21);
    public static final SwerveModulePorts FRONT_RIGHT_MOD = new SwerveModulePorts(2, 12, 22);
    public static final SwerveModulePorts BACK_LEFT_MOD = new SwerveModulePorts(3, 13, 23);
    public static final SwerveModulePorts BACK_RIGHT_MOD = new SwerveModulePorts(4, 14, 24);

    // Peripherals
    public static final int PIGEON = 0;
}
