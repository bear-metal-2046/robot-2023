package org.tahomarobotics.robot;

public final class RobotMap {
    public record SwerveModulePorts(int drive, int steer, int encoder) {}

    // Swerve Modules
    public static final SwerveModulePorts FRONT_LEFT_MOD = new SwerveModulePorts(1, 2, 3);
    public static final SwerveModulePorts FRONT_RIGHT_MOD = new SwerveModulePorts(4, 5, 6);
    public static final SwerveModulePorts BACK_LEFT_MOD = new SwerveModulePorts(7, 8, 9);
    public static final SwerveModulePorts BACK_RIGHT_MOD = new SwerveModulePorts(10, 11, 12);

    // Peripherals
    public static final int PIGEON = 0;
}
