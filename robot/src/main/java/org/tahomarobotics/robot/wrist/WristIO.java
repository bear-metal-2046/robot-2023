package org.tahomarobotics.robot.wrist;

import com.revrobotics.*;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.AutoLog;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.Robot;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.CalibrationAction;
import org.tahomarobotics.robot.util.CalibrationData;
import org.tahomarobotics.robot.util.SparkMaxHelper;

public interface WristIO {
    Logger logger = LoggerFactory.getLogger(WristIO.class);

    @AutoLog
    class WristIOInputs {
        public double position = 0;
        public double velocity = 0;
    }

    default void initialize() {
    }

    void disable();

    default void updateInputs(WristIOInputs inputs) {
        inputs.position = getPosition();
        inputs.velocity = getVelocity();
    }

    double getVelocity();

    double getPosition();

    void setPosition(double angle);

    default void calibration(CalibrationAction calibrationAction) {
    }

    class WristIOPhys implements WristIO {
        public CANSparkMax motor;
        public CalibrationData<Double> calibrationData;
        public AbsoluteEncoder absEncoder;
        public SparkMaxPIDController pidController;

        @Override
        public void initialize() {
            calibrationData = new CalibrationData<>("WristCalibration", 0d);
            motor = new CANSparkMax(RobotMap.WRIST_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
            absEncoder = motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
            pidController = motor.getPIDController();
            pidController.setFeedbackDevice(absEncoder);
            SparkMaxHelper.checkThenConfigure("Wrist Motor", logger,
                    WristConstants.createWristMotorConfig(true, calibrationData.get()), motor, absEncoder, pidController);
        }

        @Override
        public void disable() {
            pidController.setReference(0, CANSparkMax.ControlType.kDutyCycle);
        }

        @Override
        public double getVelocity() {
            return absEncoder.getVelocity();
        }

        @Override
        public double getPosition() {
            return absEncoder.getPosition();
        }

        @Override
        public void setPosition(double angle) {
            if (!Double.isNaN(angle)) {
                pidController.setReference(angle, CANSparkMax.ControlType.kPosition);
            }
        }

        public void setZeroOffset(double encoderOffset) {
            absEncoder.setZeroOffset(encoderOffset);
        }

        @Override
        public void calibration(CalibrationAction calibrationAction) {
            switch (calibrationAction) {
                // reset arm angles, so that the angle readings will indicate the unadjusted values
                case Initiate -> setZeroOffset(0);
                // reinstate old calibration offsets
                case Cancel -> setZeroOffset(calibrationData.get());
                // set and save the offsets to the negated reading from this calibration
                case Finalize -> setZeroOffset(calibrationData.set(absEncoder.getPosition()));
            }
        }
    }

    class WristIOSim implements WristIO {
        public double position, velocity;

        @Override
        public void disable() {
            position = Units.degreesToRadians(195);
        }

        @Override
        public double getVelocity() {
            return velocity;
        }

        @Override
        public double getPosition() {
            return position;
        }

        @Override
        public void setPosition(double newAngle) {
            if (!Double.isNaN(newAngle)) {
                velocity = (position - newAngle) / Robot.defaultPeriodSecs;
                position = newAngle;
            }
        }
    }
}