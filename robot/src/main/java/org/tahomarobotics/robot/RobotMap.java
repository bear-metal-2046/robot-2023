/**
 * Copyright 2023 Tahoma Robotics - http://tahomarobotics.org - Bear Metal 2046 FRC Team
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
package org.tahomarobotics.robot;

/**
 * Contains Port Mappings All Robots.
 */
public final class RobotMap {
    public record SwerveModulePorts(int drive, int steer, int encoder) {}

    public static final boolean IS_REPLAYING = false;

    /*
    Swerve Modules
     */
    public final static SwerveModulePorts FRONT_LEFT_MOD = new SwerveModulePorts(1, 11, 21);
    public final static SwerveModulePorts FRONT_RIGHT_MOD = new SwerveModulePorts(2, 12, 22);
    public final static SwerveModulePorts BACK_LEFT_MOD = new SwerveModulePorts(3, 13, 23);
    public final static SwerveModulePorts BACK_RIGHT_MOD = new SwerveModulePorts(4, 14, 24);

    public static final int SHOULDER_CANCODER = 8;
    public static final int SHOULDER_MOTOR_TOP = 6;
    public static final int SHOULDER_MOTOR_BOTTOM = 5;
    public static final int ELBOW_MOTOR = 7;
    public static final int ELBOW_CANCODER = 9;
    public static final int WRIST_MOTOR = 15;
    public static final int WRIST_CANCODER = 16;

    /*
    Climb Motors
     */
    public final static int LEFT_PAW = 31;
    public final static int RIGHT_PAW = 32;
    public final static int BEACHER = 25;

    public static final int GRABBER_MOTOR = 10;
    /*
Peripherals
 */
    public final static int PIGEON = 0;

    public final static int BLINKIN = 9;
}