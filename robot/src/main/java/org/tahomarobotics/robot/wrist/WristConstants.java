package org.tahomarobotics.robot.wrist;

import com.revrobotics.CANSparkMax;
import org.tahomarobotics.robot.util.SparkMaxConfig;

public class WristConstants {
    public static final double GEAR_RATIO = 16d/24d;
    public static SparkMaxConfig createWristMotorConfig(boolean inverted, double offset) {
        //TODO update these values for wrist; currently this is just taken from Davis' function in ArmConstants

        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.motorInverted = inverted;
        cfg.currentLimit = 40;
        cfg.positionConversionFactor = 2 * Math.PI * GEAR_RATIO;
        cfg.velocityConversionFactor = cfg.positionConversionFactor / 60d;
        cfg.encoderOffset = offset;
        cfg.idleBrake = CANSparkMax.IdleMode.kBrake;
        cfg.kP = 1.0;
        cfg.kD = 2.0;
        cfg.wrapEnabled = true;
        cfg.encoderInverted = true;

        return cfg;
    }
}
