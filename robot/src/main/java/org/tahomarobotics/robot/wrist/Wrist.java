package org.tahomarobotics.robot.wrist;


import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.SubsystemIF;
import org.tahomarobotics.robot.util.CTREPheonixHelper;
import org.tahomarobotics.robot.util.CalibrationAction;
import org.tahomarobotics.robot.util.CalibrationData;
import org.tahomarobotics.robot.util.SparkMaxHelper;

import static org.tahomarobotics.robot.wrist.WristConstants.GEAR_RATIO_STAGE_2;

public class Wrist extends SubsystemBase implements SubsystemIF {
    private static final Logger logger = LoggerFactory.getLogger(Wrist.class);
    private static final Wrist INSTANCE = new Wrist();
    CANSparkMax motor;
    CalibrationData<Double> calibrationData;
    RelativeEncoder relEncoder;
    CANCoder canCoder;
    SparkMaxPIDController pidController;
    private boolean updateEncoders = false;

    public static Wrist getInstance() {
        return INSTANCE;
    }

    private Wrist() {

        motor = new CANSparkMax(RobotMap.WRIST_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        relEncoder = motor.getEncoder();
        pidController = motor.getPIDController();
        SparkMaxHelper.checkThenConfigure("Wrist Motor", logger, WristConstants.MOTOR_CONFIG, motor, relEncoder, pidController);

        calibrationData = new CalibrationData<>("WristCalibration", 0d);
        canCoder = new CANCoder(RobotMap.WRIST_CANCODER);
        CTREPheonixHelper.checkThenConfigure("Wrist CANCoder", logger, canCoder, WristConstants.createWristEncoderConfig(calibrationData.get(), false));


        relEncoder.setPosition(canCoder.getAbsolutePosition()); // Converts from radians to degrees and from the motor axle to the absolute encoder axle

    }


    public Wrist initialize() {
        SmartDashboard.putData("Wrist Calibration", new WristCalibrationCommand());
        return this;
    }

    public void goToAngle(double angle) {
        if (!Double.isNaN(angle)) {
            pidController.setReference(angle, CANSparkMax.ControlType.kPosition);
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Wrist CANCoder Position", Units.radiansToDegrees(canCoder.getAbsolutePosition()));
        SmartDashboard.putNumber("Wrist Relative Encoder Position", Units.radiansToDegrees(relEncoder.getPosition()));
        if (updateEncoders) {
            updateEncoders = false;
            relEncoder.setPosition(canCoder.getAbsolutePosition());
        }
        if (DriverStation.isDisabled()) {
            pidController.setReference(0, CANSparkMax.ControlType.kDutyCycle);
        }
    }

    public void calibration(CalibrationAction calibrationAction) {

        switch (calibrationAction) {

            // reset arm angles, so that the angle readings will indicate the unadjusted values
            case Initiate -> setAngularOffsets(0);

            // reinstate old calibration offsets
            case Cancel -> setAngularOffsets(calibrationData.get());

            // set and save the offsets to the negated reading from this calibration
            case Finalize -> setAngularOffsets(calibrationData.set(-canCoder.getPosition()));

        }
    }

    private void setAngularOffsets(double encoderOffset) {
        canCoder.configMagnetOffset(Units.radiansToDegrees(encoderOffset) / GEAR_RATIO_STAGE_2);
        updateEncoders = true;
    }

    public double getPosition() {
        return relEncoder.getPosition();
    }
}

