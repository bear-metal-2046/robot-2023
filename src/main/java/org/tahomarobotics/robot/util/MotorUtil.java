package org.tahomarobotics.robot.util;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.REVLibError;
import org.slf4j.Logger;

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

}

