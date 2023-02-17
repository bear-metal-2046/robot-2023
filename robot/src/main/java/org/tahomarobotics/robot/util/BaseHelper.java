package org.tahomarobotics.robot.util;

import org.slf4j.Logger;

import java.util.function.Supplier;

public class BaseHelper {

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
