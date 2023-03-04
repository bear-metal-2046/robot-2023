package org.tahomarobotics.robot.wrist;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.tahomarobotics.robot.util.CalibrationAction;

public class WristCalibrationCommand extends CommandBase {
    Wrist wrist = Wrist.getInstance();
    private boolean finalized = false;
    public WristCalibrationCommand() {
        addRequirements(wrist);
    }

    private static final String FINALIZE_KEY = "Finalize";
    @Override
    public void initialize() {
        if (!DriverStation.isDisabled()) {
            cancel();
        }
        SmartDashboard.putBoolean(FINALIZE_KEY, false);
        wrist.calibration(CalibrationAction.Initiate);
    }

    @Override
    public void execute() {
        if (RobotState.isEnabled()) {
            cancel();
        }
        finalized = SmartDashboard.getBoolean(FINALIZE_KEY, false);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            wrist.calibration(CalibrationAction.Cancel);
        } else {
            wrist.calibration(CalibrationAction.Finalize);
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
