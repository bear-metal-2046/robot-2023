/**
 * Copyright 2023 Tahoma Robotics - http://tahomarobotics.org - Bear Metal 2046 FRC Team
 * <p>
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including without
 * limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
 * Software, and to permit persons to whom the Software is furnished to do so, subject to the following
 * conditions:
 * <p>
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions
 * of the Software.
 * <p>
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
 * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */
package org.tahomarobotics.robot.climb;

import edu.wpi.first.math.system.plant.DCMotor;
import org.tahomarobotics.robot.ident.RobotIdentity;
import org.tahomarobotics.robot.util.SparkMaxConfig;

public class ClimbConstants {

    public static final DCMotor DC_MOTOR = DCMotor.getNEO(1);

    public final static int FULL_CURRENT_LIMIT = 50;
    public final static int ZERO_CURRENT_LIMIT = 20;

    public record PawHardwareConfig(double gearReduction, double zeroReference) {
    }

    public static final PawHardwareConfig PAW_HARDWARE_CONFIG = switch (RobotIdentity.getInstance().getRobotID()) {
        // no climb hardware
        case PROTOTYPE -> new PawHardwareConfig(1, 0);

        // initial climb hardware
        case ALPHA -> new PawHardwareConfig(10d / 60d * 24d / 56d * 16d / 60d, 0);

        // final climb hardware
        case PRACTICE -> new PawHardwareConfig(10d / 60d * 24d / 72d * 16d / 60d, 0);
        case COMPETITION -> new PawHardwareConfig(10d / 60d * 24d / 72d * 16d / 60d, 0);
    };

    private final static double POSITION_FACTOR = 2 * Math.PI * PAW_HARDWARE_CONFIG.gearReduction;
    private final static double VELOCITY_FACTOR = POSITION_FACTOR / 60;
    public final static double kV = 1d / (DC_MOTOR.KvRadPerSecPerVolt * PAW_HARDWARE_CONFIG.gearReduction);

    //TODO Calculate kA for Paw
    public final static double kA = 0;

    public static SparkMaxConfig createPawConfig(int id, boolean inverted) {
        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.canId = id;
        cfg.motorInverted = inverted;
        cfg.kP = 1;
        cfg.positionConversionFactor = POSITION_FACTOR;
        cfg.velocityConversionFactor = VELOCITY_FACTOR;
        cfg.currentLimit = FULL_CURRENT_LIMIT;
        return cfg;
    }

    public static SparkMaxConfig createBeacherConfig(int id, boolean inverted) {
        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.canId = id;
        cfg.motorInverted = inverted;
        return cfg;
    }
}
