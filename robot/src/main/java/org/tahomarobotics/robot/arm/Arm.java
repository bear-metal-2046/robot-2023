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

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.ident.RobotIdentity;
import org.tahomarobotics.robot.util.CTREPheonixHelper;
import org.tahomarobotics.robot.util.CalibrationAction;
import org.tahomarobotics.robot.util.CalibrationData;
import org.tahomarobotics.robot.util.SparkMaxHelper;

import java.io.Serializable;

import static org.tahomarobotics.robot.arm.ArmConstants.*;

public class Arm extends SubsystemBase implements ArmSubsystemIF {
    private static final Logger logger = LoggerFactory.getLogger(Arm.class);
    private boolean angleDisplayEnabled = true;

    // Configures the Arm according to the current RobotID.
    private static final ArmSubsystemIF INSTANCE = switch (RobotIdentity.getInstance().getRobotID()) {
        // empty arm
        case PROTOTYPE -> new ArmSubsystemIF() {};
        // configure Rev Swerve
        case ALPHA, PRACTICE, COMPETITION -> new Arm();
    };

    public static ArmSubsystemIF getInstance() {
        return INSTANCE;
    }

    private record EncoderOffsets(double shoulder, double elbow) implements Serializable {
        @Override
        public String toString() {
            return String.format("EncoderOffsets{ shoulder=%7.4f° elbow=%7.4f° }", Units.radiansToDegrees(shoulder), Units.radiansToDegrees(elbow));
        }
    }
    private final CalibrationData<EncoderOffsets> calibrationData;    private final CANSparkMax shoulderMotor;
    private final CANSparkMax shoulderFollower;
    private final CANSparkMax elbowMotor;
    private final CANCoder shoulderEncoder;
    private final CANCoder elbowEncoder;
    private final PIDController shoulderPIDController;
    private final PIDController elbowPIDController;
    private final ArmFeedForward feedForward = new ArmFeedForward();
    private final ArmKinematics kinematics = new ArmKinematics();

    private record ArmMechanism (Mechanism2d mech, MechanismLigament2d upperArm, MechanismLigament2d foreArm) {}

    private final ArmMechanism armMechanism;

    private ArmState desiredState = null;

    private final double voltages[] = new double[4];

    private final static ArmState initialState = new ArmState(0,
            new ArmState.JointState(Units.degreesToRadians(36.3), 0, 0),
            new ArmState.JointState(Units.degreesToRadians(-136.7), 0, 0));

    private Arm() {

        // Shoulder Motor

        shoulderMotor = new CANSparkMax(RobotMap.SHOULDER_MOTOR_TOP, CANSparkMaxLowLevel.MotorType.kBrushless);
        SparkMaxHelper.checkThenConfigure("Shoulder Leader Motor", logger, ArmConstants.CONFIG_SHOULDER_MOTOR, shoulderMotor);

        shoulderFollower = new CANSparkMax(RobotMap.SHOULDER_MOTOR_BOTTOM, CANSparkMaxLowLevel.MotorType.kBrushless);
        SparkMaxHelper.checkThenConfigure("Shoulder Follower Motor", logger, ArmConstants.CONFIG_SHOULDER_MOTOR, shoulderFollower);
        //shoulderFollower.follow(shoulderMotor);


        // Elbow Motor
        elbowMotor = new CANSparkMax(RobotMap.ELBOW_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        SparkMaxHelper.checkThenConfigure("Elbow Motor", logger, ArmConstants.CONFIG_ELBOW_MOTOR, elbowMotor);

        // calibration offsets from file
        calibrationData = new CalibrationData<>("ArmCalibration", new EncoderOffsets(0d, 0d));
        EncoderOffsets encoderOffsets = calibrationData.get();

        // Shoulder Encoder
        shoulderEncoder = new CANCoder(RobotMap.SHOULDER_CANCODER);
        CANCoderConfiguration shoulderEncoderConfig = ArmConstants.createArmEncoderConfig(encoderOffsets.shoulder(),
                ARM_PHYSICAL_PROPERTIES.upperArm().encoderInverted());
        CTREPheonixHelper.checkThenConfigure("Shoulder Encoder", logger, shoulderEncoder, shoulderEncoderConfig);

        // Elbow Encoder
        elbowEncoder = new CANCoder(RobotMap.ELBOW_CANCODER);
        CANCoderConfiguration elbowEncoderConfig = ArmConstants.createArmEncoderConfig(encoderOffsets.elbow(),
                ARM_PHYSICAL_PROPERTIES.foreArm().encoderInverted());
        CTREPheonixHelper.checkThenConfigure("Elbow Encoder", logger, elbowEncoder, elbowEncoderConfig);

        // Shoulder PID Controller
        shoulderPIDController = new PIDController(ArmConstants.CONFIG_SHOULDER_MOTOR.kP,ArmConstants.CONFIG_SHOULDER_MOTOR.kI,ArmConstants.CONFIG_SHOULDER_MOTOR.kD);

        // Elbow PID Controller
        elbowPIDController = new PIDController(ArmConstants.CONFIG_ELBOW_MOTOR.kP,ArmConstants.CONFIG_ELBOW_MOTOR.kI,ArmConstants.CONFIG_ELBOW_MOTOR.kD);

        shoulderPIDController.setIntegratorRange(-2.0, 2.0);
        elbowPIDController.setIntegratorRange(-2.0, 2.0);

        Mechanism2d mech = new Mechanism2d(HORIZONTAL_EXTENSION_FRONT_LIMIT + 2 * FRAME_HALF_SIZE + HORIZONTAL_EXTENSION_REAR_LIMIT ,
                VERTICAL_EXTENSION_UPPER_LIMIT);
        MechanismRoot2d root = mech.getRoot("shoulder", FRAME_HALF_SIZE + HORIZONTAL_EXTENSION_REAR_LIMIT + SHOULDER_AXIS.getX(), SHOULDER_AXIS.getY());
        MechanismLigament2d upperArm = root.append(new MechanismLigament2d("upper-arm",
                ArmConstants.ARM_PHYSICAL_PROPERTIES.upperArm().length(), 0));
        MechanismLigament2d foreArm = upperArm.append(new MechanismLigament2d("fore-arm",
                ArmConstants.ARM_PHYSICAL_PROPERTIES.foreArm().length(), -90, 6, new Color8Bit(Color.kPurple)));
        armMechanism = new ArmMechanism(mech, upperArm, foreArm);

        SmartDashboard.putData("Arm", mech);
    }

    @Override
    public ArmSubsystemIF initialize() {


        SmartDashboard.putData("Arm to Stow",
                new ArmMoveCommand(ArmMovements.START_TO_STOW));

        SmartDashboard.putData("Position to Stow",
                ArmMovements.createPositionToStowCommand());

        SmartDashboard.putData("Stow to Up Collect",
                new ArmMoveCommand(ArmMovements.STOW_TO_CONE_COLLECT));

        SmartDashboard.putData("Up Collect to Stow",
                new ArmMoveCommand(ArmMovements.CONE_COLLECT_TO_STOW));

        SmartDashboard.putData("Stow to Down Collect",
                new ArmMoveCommand(ArmMovements.STOW_TO_CUBE_COLLECT));

        SmartDashboard.putData("Down Collect to Stow",
                new ArmMoveCommand(ArmMovements.CUBE_COLLECT_TO_STOW));

        SmartDashboard.putData("Stow to Feeder Collect",
                new ArmMoveCommand(ArmMovements.STOW_TO_CONE_FEEDER_COLLECT));

        SmartDashboard.putData("Feeder Collect to Stow",
                new ArmMoveCommand(ArmMovements.CONE_FEEDER_COLLECT_TO_STOW));



        SmartDashboard.putData("Calibrate Arm", new ArmCalibrationCommand());
        return this;
    }

    @Override
    public ArmState getCurrentArmState() {
        return new ArmState(0d,
                new ArmState.JointState(shoulderEncoder.getAbsolutePosition(), shoulderEncoder.getVelocity(), 0),
                new ArmState.JointState(elbowEncoder.getAbsolutePosition(), elbowEncoder.getVelocity(), 0));
    }

    @Override
    public Translation2d getCurrentPosition() {
        return new ArmKinematics().forwardKinematics(getCurrentArmState());
    }

    @Override
    public void setArmState(ArmState desiredState) {
        if (desiredState != null) {
            this.desiredState = desiredState;
        } else {
            logger.error("desired state set to null");
        }
    }

    @Override
    public void periodic() {

        ArmState currentState = getCurrentArmState();

        if (angleDisplayEnabled) {
            SmartDashboard.putNumber("Shoulder Angle", Units.radiansToDegrees(currentState.shoulder.position()));
            SmartDashboard.putNumber("Elbow Angle", Units.radiansToDegrees(currentState.elbow.position()));
        }

        double shoulderVoltage = 0;
        double elbowVoltage = 0;

        // disable arm when robot is transitioned to disable
        // this is to ensure that the arm doesn't move until an arm movement action
        // is initiated by autonomous or drive team
        if (RobotState.isDisabled() && desiredState != null) {
            desiredState = null;
        }

        // arm movement is initiated by setting the desired state
        if (desiredState != null) {

            ArmFeedForward.FeedForwardVoltages ffVoltages = feedForward.calculate(desiredState, currentState);
            double shoulderFeedforwardVoltage = ffVoltages.shoulder();
            double elbowFeedforwardVoltage = ffVoltages.elbow();
            voltages[2] = shoulderFeedforwardVoltage;
            voltages[3] = elbowFeedforwardVoltage;

            SmartDashboard.putNumber("Shoulder FF Voltage", shoulderFeedforwardVoltage);
            SmartDashboard.putNumber("Elbow FF Voltage", elbowFeedforwardVoltage);

            double shoulderFeedbackVoltage = shoulderPIDController.calculate(shoulderEncoder.getPosition(), desiredState.shoulder.position());
            double elbowFeedbackVoltage = elbowPIDController.calculate(elbowEncoder.getPosition(), desiredState.elbow.position());

            SmartDashboard.putNumber("Shoulder Diff", shoulderEncoder.getPosition() - desiredState.shoulder.position());
            SmartDashboard.putNumber("Elbow Diff", elbowEncoder.getPosition() - desiredState.elbow.position());

            // limit feed-back voltages
            // BUT DO NOT ALLOW FULL VOLTAGE to be applied to motor if there are large errors
            shoulderFeedbackVoltage = MathUtil.clamp(shoulderFeedbackVoltage, -MAX_PID_VOLTAGE, MAX_PID_VOLTAGE);
            elbowFeedbackVoltage = MathUtil.clamp(elbowFeedbackVoltage, -MAX_PID_VOLTAGE, MAX_PID_VOLTAGE);

            // apply voltages to output
            shoulderVoltage = shoulderFeedforwardVoltage + shoulderFeedbackVoltage;
            elbowVoltage = elbowFeedforwardVoltage + elbowFeedbackVoltage;

            SmartDashboard.putNumber("Desired Shoulder Angle", Units.radiansToDegrees(desiredState.shoulder.position()));
            SmartDashboard.putNumber("Desired Elbow Angle", Units.radiansToDegrees(desiredState.elbow.position()));
        }

        // power the motors
        shoulderMotor.setVoltage(shoulderVoltage);
        shoulderFollower.setVoltage(shoulderVoltage);
        elbowMotor.setVoltage(elbowVoltage);
        voltages[0] = shoulderVoltage;
        voltages[1] = elbowVoltage;

        SmartDashboard.putNumber("Shoulder Voltage", shoulderVoltage);
        SmartDashboard.putNumber("Elbow Voltage", elbowVoltage);

        Translation2d position = kinematics.forwardKinematics(getCurrentArmState());
        SmartDashboard.putNumber("Arm X", Units.metersToInches(position.getX()));
        SmartDashboard.putNumber("Arm Y", Units.metersToInches(position.getY()));


        armMechanism.upperArm.setAngle(Units.radiansToDegrees(currentState.shoulder.position()));
        armMechanism.foreArm.setAngle(Units.radiansToDegrees(currentState.elbow.position()));
    }

    /**
     * Calibration of the arm is accomplished by:
     *   1) Initiate calibration mode
     *   2) Move shoulder joint to vertical
     *   3) Move elbow joint to forward horizontal
     *   4) While holding arms, Finalize calibration
     *   5) Check reading
     *
     * @param calibrationAction - action from command
     */
    @Override
    public void calibration(CalibrationAction calibrationAction) {

        switch (calibrationAction) {

            // reset arm angles, so that the angle readings will indicate the unadjusted values
            case Initiate -> {
                angleDisplayEnabled = true;
                setAngularOffsets(new EncoderOffsets(Math.PI / 2 * (shoulderEncoder.configGetSensorDirection() ? 1d : -1d), 0),
                        CANSparkMax.IdleMode.kCoast);
            }
            // reinstate old calibration offsets
            case Cancel -> setAngularOffsets(calibrationData.get(),
                    CANSparkMax.IdleMode.kBrake);

            // set and save the offsets to the negated reading from this calibration
            case Finalize -> setAngularOffsets(calibrationData.set(new EncoderOffsets(
                            shoulderEncoder.getAbsolutePosition() * (shoulderEncoder.configGetSensorDirection() ? 1d : -1d),
                            elbowEncoder.getAbsolutePosition() * (elbowEncoder.configGetSensorDirection() ? 1d : -1d))),
                    CANSparkMax.IdleMode.kBrake);
        }
    }

    @Override
    public double[] getVoltages() {
        return voltages;
    }


    private void setAngularOffsets(EncoderOffsets encoderOffsets, CANSparkMax.IdleMode mode) {
        shoulderMotor.setIdleMode(mode);
        elbowMotor.setIdleMode(mode);

        shoulderEncoder.configMagnetOffset(Units.radiansToDegrees(encoderOffsets.shoulder));
        elbowEncoder.configMagnetOffset(Units.radiansToDegrees(encoderOffsets.elbow));
    }

    @Override
    public void simulationPeriodic() {
        ArmState state = desiredState == null ? initialState : desiredState;
        updateEncoder(shoulderEncoder, state.shoulder);
        updateEncoder(elbowEncoder, state.elbow);
    }

    private void updateEncoder(CANCoder encoder, ArmState.JointState state) {
        var sim = encoder.getSimCollection();

        sim.setRawPosition((int)(state.position() / encoder.configGetFeedbackCoefficient() * (encoder.configGetSensorDirection() ? -1d : 1d)));
        sim.setVelocity((int)(state.position() * 10 / encoder.configGetFeedbackCoefficient() * (encoder.configGetSensorDirection() ? -1d : 1d)));
    }
}
