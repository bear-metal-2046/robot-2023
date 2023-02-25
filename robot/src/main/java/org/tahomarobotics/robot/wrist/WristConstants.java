package org.tahomarobotics.robot.wrist;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.util.Units;
import org.tahomarobotics.robot.arm.ArmConstants;
import org.tahomarobotics.robot.util.SparkMaxConfig;

public class WristConstants {
    public static final SparkMaxConfig MOTOR_CONFIG = createWristMotorConfig(false, new ArmConstants.PIDGains(.5, 0, 0));

    public static final double GEAR_RATIO_STAGE_1 = 12d/72d;
    public static final double GEAR_RATIO_STAGE_2 = 18d/24d;
    public static final double MAX_ACCEL = 1;
    public static SparkMaxConfig createWristMotorConfig(boolean inverted, ArmConstants.PIDGains pidGains) {
        //TODO update these values for wrist; currently this is just taken from Davis' function in ArmConstants

        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.motorInverted = inverted;
        cfg.currentLimit = 10;
        cfg.positionConversionFactor = 2 * Math.PI * GEAR_RATIO_STAGE_1 * GEAR_RATIO_STAGE_2;
        cfg.velocityConversionFactor = cfg.positionConversionFactor / 60d;
        cfg.idleBrake = CANSparkMax.IdleMode.kBrake;
        cfg.kP = pidGains.kP();
        cfg.kI = pidGains.kI();
        cfg.kD = pidGains.kD();

        return cfg;
    }

    public static CANCoderConfiguration createWristEncoderConfig(double angularOffset, boolean inverted) {
        CANCoderConfiguration cfg = new CANCoderConfiguration();
        //TODO update these values for wrist; currently this is just taken from Davis' function in ArmConstants

        cfg.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
        cfg.sensorDirection = inverted;
        cfg.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        cfg.magnetOffsetDegrees = Units.radiansToDegrees(angularOffset) / GEAR_RATIO_STAGE_2;
        cfg.sensorCoefficient = Math.PI * 2d / 4096d * GEAR_RATIO_STAGE_2;
        cfg.unitString = "radians";
        cfg.sensorTimeBase = SensorTimeBase.PerSecond;

        return cfg;
    }
}
