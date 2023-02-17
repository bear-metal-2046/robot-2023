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
package org.tahomarobotics.robot.util;

import com.revrobotics.CANSparkMax;

public class SparkMaxConfig {

    // motor controller
    public int canId = 0;
    public boolean motorInverted = false;

    public int currentLimit = 80;
    public CANSparkMax.IdleMode idleBrake = CANSparkMax.IdleMode.kBrake;
    public double compensationNominal = 12.0;

    // encoder
    public double positionConversionFactor = 1.0;
    public double velocityConversionFactor = 1.0;
    public boolean encoderInverted = false;
    public double encoderOffset = 0;

    // PID controller
    public double kP = 1.0;
    public double kI = 0.0;
    public double kD = 0.0;
    public double kFF = 0.0;
    public boolean wrapEnabled = false;
    public double wrapMin = 0.0;
    public double wrapMax = Math.PI * 2;

}

