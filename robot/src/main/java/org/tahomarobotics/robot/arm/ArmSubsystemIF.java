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
package org.tahomarobotics.robot.arm;

import edu.wpi.first.math.geometry.Translation2d;
import org.tahomarobotics.robot.SubsystemIF;
import org.tahomarobotics.robot.util.CalibrationAction;

public interface ArmSubsystemIF extends SubsystemIF {

    record ArmElectricalInfo(double shoulderVoltage, double elbowVoltage, double shoulderCurrent, double elbowCurrent) {}

    default ArmState getCurrentArmState() { return new ArmState(); }

    default Translation2d getCurrentPosition() { return new Translation2d(0,0); }

    default void setArmState(ArmState desiredState) {}

    default void calibration(CalibrationAction calibrationAction) {}

    default boolean isAtPosition() {return true;}

    default ArmElectricalInfo getArmElectricalInfo() { return new ArmElectricalInfo(0,0,0,0);}
}
