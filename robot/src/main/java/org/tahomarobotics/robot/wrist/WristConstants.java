package org.tahomarobotics.robot.wrist;

import com.revrobotics.CANSparkMax;
import org.tahomarobotics.robot.util.SparkMaxConfig;

public class WristConstants {
    public static final double GEAR_RATIO_STAGE_1 = 12d/72d;
    public static final double GEAR_RATIO_STAGE_2 = 18d/24d;
    public static final double MAX_ACCEL = 1;
    public static SparkMaxConfig createWristMotorConfig(boolean inverted, double offset) {
        //TODO update these values for wrist; currently this is just taken from Davis' function in ArmConstants

        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.motorInverted = inverted;
        cfg.currentLimit = 20;
        cfg.positionConversionFactor = 2 * Math.PI;
        cfg.velocityConversionFactor = cfg.positionConversionFactor / 60d;
        cfg.encoderOffset = offset;
        cfg.idleBrake = CANSparkMax.IdleMode.kBrake;
        cfg.kP = 1;
        cfg.wrapEnabled = true;
        cfg.encoderInverted = true;

        return cfg;
    }
}
