package org.tahomarobotics.robot.arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.tahomarobotics.robot.util.CalibrationAction;

public class ArmCalibrationCommand extends CommandBase {
    ArmSubsystemIF arm = Arm.getInstance();
    private boolean finalized = false;
    public ArmCalibrationCommand() {
        addRequirements(arm);
    }

    private static final String FINALIZE_KEY = "Finalize";
    @Override
    public void initialize() {
        if (!DriverStation.isDisabled()) {
            cancel();
        }
        SmartDashboard.putBoolean(FINALIZE_KEY, false);
        arm.calibration(CalibrationAction.Initiate);
    }

    @Override
    public void execute() {
        if (!RobotState.isDisabled()) {
            cancel();
        }
        finalized = SmartDashboard.getBoolean(FINALIZE_KEY, false);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            arm.calibration(CalibrationAction.Cancel);
        } else {
            arm.calibration(CalibrationAction.Finalize);
        }
    }

    @Override
    public boolean isFinished() {
        return finalized;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
