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
package org.tahomarobotics.robot.wrist;

import edu.wpi.first.math.util.Units;

public enum WristPosition {
    STOW(Units.degreesToRadians(128d)),
    CUBE_COLLECT(Units.degreesToRadians(89.5)),
    CONE_COLLECT(Units.degreesToRadians(100.5)),
    CUBE_FEEDER_COLLECT(Units.degreesToRadians(83d)),
    CONE_FEEDER_COLLECT(Units.degreesToRadians(130d)),
    MID_BOX_PLACE(Units.degreesToRadians(29d)),
    HIGH_BOX_PLACE(Units.degreesToRadians(-35d)),
    MID_POLE_PLACE(Units.degreesToRadians(-58d)),
    HIGH_POLE_PLACE(Units.degreesToRadians(-52d));

    public final double angle;
    WristPosition(double angle) {
        this.angle = angle;
    }
}
