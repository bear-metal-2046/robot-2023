package org.tahomarobotics.robot.util;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.*;
import java.util.Scanner;

/**
 * DoubleProperty Class
 * Persistently saved property.
 */
public class DoubleProperty {
    private final static Logger logger = LoggerFactory.getLogger(DoubleProperty.class);

    private final String name;
    private double value;
    private final double defaultValue;

    public DoubleProperty(String name, double defaultValue) {
        this.name = name;
        this.defaultValue = defaultValue;
        this.value = deserializeValue();
    }

    public void setValue(double value) {
        this.value = value;
        serialize();
    }

    public double getValue() {
        return value;
    }

    public void serialize() {
        try (FileWriter fw = new FileWriter("/home/lvuser/" + name)) {
            fw.write(String.valueOf(this.value));
        } catch (Exception ignore) {}
    }

    public double deserializeValue() {
        try (Scanner s = new Scanner(new File("/home/lvuser/" + name))) {
            return s.nextDouble();
        } catch (Exception ignore) {
            logger.error("No Calibration File Found For " + name);
            return this.defaultValue;
        }
    }
}
