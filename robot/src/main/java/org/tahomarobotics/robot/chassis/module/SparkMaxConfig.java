package org.tahomarobotics.robot.chassis.module;

import com.revrobotics.CANSparkMax;

public class SparkMaxConfig {

    // motor controller
    int canId = 0;
    boolean motorInverted = false;
    int currentLimit = 80;
    CANSparkMax.IdleMode idleBrake = CANSparkMax.IdleMode.kBrake;
    double compensationNominal = 12.0;

    // encoder
    double positionConversionFactor = 1.0;
    double velocityConversionFact = 1.0;
    boolean encoderInverted = false;
    double encoderOffset = 0;

    // PID controller
    double kP = 0.0;
    double kI = 0.0;
    double kD = 0.0;
    double kFF = 0.0;
    boolean wrapEnabled = false;
    double wrapMin = 0.0;
    double wrapMax = Math.PI * 2;

}
