package org.tahomarobotics.robot.arm;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.tahomarobotics.robot.util.CalibrationAction;

import java.util.function.BooleanSupplier;

public class ArmCalibrationCommand extends CommandBase {
    private final ArmSubsystemIF arm;

    private final BooleanSupplier finalizeInput;

    private final BooleanConsumer finializeUpdate;
    private boolean finalized = false;
    private static final String FINALIZE_KEY = "Finalize";


    public ArmCalibrationCommand() {
        this(Arm.getInstance(),
                () -> SmartDashboard.getBoolean(FINALIZE_KEY, false),
                (f) -> SmartDashboard.putBoolean(FINALIZE_KEY, false));
    }
    public ArmCalibrationCommand(ArmSubsystemIF arm, BooleanSupplier finalizeInitial, BooleanConsumer finializeUpdate) {
        this.arm = arm;
        this.finalizeInput = finalizeInitial;
        this.finializeUpdate = finializeUpdate;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        if (!DriverStation.isDisabled()) {
            cancel();
        }
        finializeUpdate.accept(false);
        arm.calibration(CalibrationAction.Initiate);
    }

    @Override
    public void execute() {
        if (!RobotState.isDisabled()) {
            cancel();
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            arm.calibration(CalibrationAction.Cancel);
        } else {
            arm.calibration(CalibrationAction.Finalize);
        }
        finializeUpdate.accept(false);
    }

    @Override
    public boolean isFinished() {
        return finalizeInput.getAsBoolean();
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
