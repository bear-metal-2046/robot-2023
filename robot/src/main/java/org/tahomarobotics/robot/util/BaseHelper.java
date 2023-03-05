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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.slf4j.Logger;

import java.util.function.Supplier;

public class BaseHelper {

    protected static final String FORCE_CONFIGURE = "Force Configure";

    protected static boolean forceConfigure = SmartDashboard.getBoolean(FORCE_CONFIGURE, false);
    private static final double EPSILON = 0.000001d;

    /**
     * Helper to compare floating point
     */
    public static boolean isEqual(double a, double b, double eps) {
        return Math.abs(a-b) < eps;
    }

    /**
     * Helper to compare floating point
     */
    public static boolean isEqual(double a, double b) {
        return isEqual(a, b, EPSILON);
    }

    /**
     * Compares and logs if appropriate
     */
    public static <T> boolean isDifferent(Logger logger, String name, Supplier<T> getter, T expected) {
        var value = getter.get();
        boolean different = expected instanceof Number ? ! isEqual((Double)value, (Double)expected) : !value.equals(expected);
        if (different) {
            logger.warn("Parameter " + name + " was " + value + " when " + expected + " was expected.");
        }
        return different;
    }

    public static <T> boolean isDifferent(Logger logger, String name, Supplier<T> getter, T expected, double epsilon) {
        var value = getter.get();
        boolean different = expected instanceof Number ? ! isEqual((Double)value, (Double)expected, epsilon) : !value.equals(expected);
        if (different) {
            logger.warn("Parameter " + name + " was " + value + " when " + expected + " was expected.");
        }
        return different;
    }
}
