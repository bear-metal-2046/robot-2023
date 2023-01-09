package org.tahomarobotics.robot.util;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.REVLibError;
import edu.wpi.first.math.util.Units;
import org.slf4j.Logger;
import org.tahomarobotics.robot.chassis.ChassisConstants;

public class MotorUtil {

    private static final int CAN_TIMEOUT_MS = 500;
    private static final int STATUS_FRAME_GENERAL_PERIOD_MS = 250;

    public static boolean reduceRateGeneralStatus(TalonFX motor) {
        ErrorCode rc = motor.setStatusFramePeriod(
                StatusFrameEnhanced.Status_1_General,
                STATUS_FRAME_GENERAL_PERIOD_MS,
                CAN_TIMEOUT_MS);

        if (rc != ErrorCode.OK) {
            LoggerManager.error("Failed to reduce general status rate for motor " + motor.getDeviceID() + ": " + rc.toString());
            return false;
        }
        return true;
    }

    public static boolean reduceRateGeneralStatus(CANSparkMax motor) {

        REVLibError rc = motor.setPeriodicFramePeriod(
                CANSparkMaxLowLevel.PeriodicFrame.kStatus0,
                STATUS_FRAME_GENERAL_PERIOD_MS);

        if (rc != REVLibError.kOk) {
            LoggerManager.error("Failed to reduce general status rate for motor " + motor.getDeviceId() + ": " + rc.toString());
            return false;
        }
        return true;
    }

    public static boolean reduceRateStatus1(CANSparkMax motor) {

        REVLibError rc = motor.setPeriodicFramePeriod(
                CANSparkMaxLowLevel.PeriodicFrame.kStatus1,
                100);

        if (rc != REVLibError.kOk) {
            LoggerManager.error("Failed to reduce general status rate for motor " + motor.getDeviceId() + ": " + rc.toString());
            return false;
        }
        return true;
    }

    public static boolean reduceRateStatus2(CANSparkMax motor) {

        REVLibError rc = motor.setPeriodicFramePeriod(
                CANSparkMaxLowLevel.PeriodicFrame.kStatus2,
                100);

        if (rc != REVLibError.kOk) {
            LoggerManager.error("Failed to reduce general status rate for motor " + motor.getDeviceId() + ": " + rc.toString());
            return false;
        }
        return true;
    }

    public static double dtLinearToMotorRot_rad(double linear_m_in){
        return linear_m_in / (Units.inchesToMeters(ChassisConstants.WHEEL_RADIUS)) * ChassisConstants.DRIVE_REDUCTION_MK4I_L2;
    }

    public static double dtMotorRotToLinear_m(double motor_rad_in){
        return motor_rad_in * (Units.inchesToMeters(ChassisConstants.WHEEL_RADIUS)) / ChassisConstants.DRIVE_REDUCTION_MK4I_L2;
    }
}

