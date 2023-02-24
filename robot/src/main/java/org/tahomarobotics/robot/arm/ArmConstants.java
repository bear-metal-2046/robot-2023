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

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import org.tahomarobotics.robot.ident.RobotIdentity;
import org.tahomarobotics.robot.util.SparkMaxConfig;

public class ArmConstants {

    public record ArmLinkPhysicalProperties(double length, double mass, double inertiaCenterMass, double locationCenterMass,
                                            double gearReduction, boolean encoderInverted, boolean motorInverted) { }

    public record ArmPhysicalProperties (ArmLinkPhysicalProperties upperArm, ArmLinkPhysicalProperties foreArm) {}

    private static double lbsInchSqTokgsMeterSq(double value) { return Units.inchesToMeters(Units.inchesToMeters(Units.lbsToKilograms(value))); }


    public static final ArmPhysicalProperties ARM_PHYSICAL_PROPERTIES =

            switch (RobotIdentity.getInstance().getRobotID()) {
                case PROTOTYPE -> new ArmPhysicalProperties(
                        new ArmLinkPhysicalProperties(30, 0, 0, 0, 1, false, false),
                        new ArmLinkPhysicalProperties(30, 0, 0, 0, 1, false, false));

                case ALPHA -> new ArmPhysicalProperties(
                        new ArmLinkPhysicalProperties(
                                Units.inchesToMeters(30),
                                Units.lbsToKilograms(4.232),
                                lbsInchSqTokgsMeterSq(448.225),
                                Units.inchesToMeters(11.86),
                                10d / 64d * 24d / 78d * 12d / 48d, false, false),

                        new ArmLinkPhysicalProperties(
                                Units.inchesToMeters(32),
                                Units.lbsToKilograms(6.967),
                                lbsInchSqTokgsMeterSq(985.969 ),
                                Units.inchesToMeters(28.55),
                                10d / 64d * 32d / 72d * 16d / 60d * 36d / 60d, false, true));

                case COMPETITION, PRACTICE -> new ArmPhysicalProperties(
                        new ArmLinkPhysicalProperties(
                                Units.inchesToMeters(33.7),
                                Units.lbsToKilograms(5.490),
                                lbsInchSqTokgsMeterSq(826.435),
                                Units.inchesToMeters(13.006),
                                11d / 62d * 30d / 84d * 12d / 60d, true, true),

                        new ArmLinkPhysicalProperties(
                                Units.inchesToMeters(31.7),
                                Units.lbsToKilograms(11.144),
                                lbsInchSqTokgsMeterSq(2004.379),
                                Units.inchesToMeters(27.381),
                                11d / 72d * 24d / 72d * 22d / 50d * 32d / 62d, false, false));
            };

    public static final double MAX_PID_VOLTAGE = 4.0;

    public record PIDGains(double kP, double kI, double kD){}

    public static final PIDGains SHOULDER_PID_GAINS = new PIDGains(16, 0, 0);
    public static final PIDGains ELBOW_PID_GAINS = new PIDGains(16, 0, 0);

    private static SparkMaxConfig createArmMotorConfig(boolean inverted, PIDGains pidGains) {
        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.motorInverted = inverted;
        cfg.currentLimit = 20;
        cfg.positionConversionFactor = 1;
        cfg.velocityConversionFactor = 1;
        cfg.idleBrake = CANSparkMax.IdleMode.kBrake;
        cfg.kP = pidGains.kP;
        cfg.kI = pidGains.kI;
        cfg.kD = pidGains.kD;
        return cfg;
    }
    public static final SparkMaxConfig CONFIG_SHOULDER_MOTOR = createArmMotorConfig(
            ARM_PHYSICAL_PROPERTIES.upperArm.motorInverted,
            SHOULDER_PID_GAINS);

    public static final SparkMaxConfig CONFIG_ELBOW_MOTOR = createArmMotorConfig(
            ARM_PHYSICAL_PROPERTIES.foreArm.motorInverted,
            ELBOW_PID_GAINS);


    public static CANCoderConfiguration createArmEncoderConfig(double angularOffset, boolean inverted) {
        CANCoderConfiguration cfg = new CANCoderConfiguration();

        cfg.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
        cfg.sensorDirection = inverted;
        cfg.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        cfg.magnetOffsetDegrees = Units.radiansToDegrees(angularOffset);
        cfg.sensorCoefficient = Math.PI * 2d / 4096.0;
        cfg.unitString = "radians";
        cfg.sensorTimeBase = SensorTimeBase.PerSecond;

        return cfg;
    }

    static final double FRAME_HALF_SIZE = Units.inchesToMeters(14);
    public static final double VERTICAL_EXTENSION_UPPER_LIMIT = Units.inchesToMeters(78);
    private static final double VERTICAL_EXTENSION_LOWER_LIMIT = Units.inchesToMeters(1);
    static final double HORIZONTAL_EXTENSION_FRONT_LIMIT = Units.inchesToMeters(48);
    static final double HORIZONTAL_EXTENSION_REAR_LIMIT = Units.inchesToMeters(48);

    public static final Translation2d SHOULDER_AXIS = new Translation2d(Units.inchesToMeters(-9), Units.inchesToMeters(20.743));

    public record Range(double min, double max) {}

    // Vertical extension shifted for the height of shoulder axis
    public static final Range VERTICAL_LIMITS = new Range(
            ArmConstants.VERTICAL_EXTENSION_LOWER_LIMIT - ArmConstants.SHOULDER_AXIS.getY(),
            ArmConstants.VERTICAL_EXTENSION_UPPER_LIMIT - ArmConstants.SHOULDER_AXIS.getY());

    public static final Range HORIZONTAL_LIMITS = new Range(
            -ArmConstants.HORIZONTAL_EXTENSION_REAR_LIMIT - ArmConstants.FRAME_HALF_SIZE + ArmConstants.SHOULDER_AXIS.getY(),
            ArmConstants.HORIZONTAL_EXTENSION_FRONT_LIMIT + ArmConstants.FRAME_HALF_SIZE + ArmConstants.SHOULDER_AXIS.getY()
    );

    public static final Range TOWER_EXCLUSION_VERTICAL = new Range( -SHOULDER_AXIS.getY() ,Units.inchesToMeters(2));
    public static final Range TOWER_EXCLUSION_HORIZONTAl = new Range(Units.inchesToMeters(-7.5), Units.inchesToMeters(-7.5));

    public static final Range SHOULDER_LIMITS = new Range( Units.degreesToRadians(37.4), Units.degreesToRadians(147.8) );
    public static final Range ELBOW_LIMITS = new Range( Units.degreesToRadians(-160d), Units.degreesToRadians(0d) );

}