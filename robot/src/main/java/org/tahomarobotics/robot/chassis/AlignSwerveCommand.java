package org.tahomarobotics.robot.chassis;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AlignSwerveCommand extends CommandBase {

    private final Chassis chassis = Chassis.getInstance();
    private boolean finalized = false;

    public AlignSwerveCommand() {
        addRequirements(chassis);
    }

    private static final String FINALIZE_KEY = "Finalize";

    @Override
    public void initialize() {
        SmartDashboard.putBoolean(FINALIZE_KEY, false);
        chassis.zeroOffsets();
    }

    @Override
    public void execute() {
        chassis.displayAbsolutePositions();
        finalized = SmartDashboard.getBoolean(FINALIZE_KEY, false);
    }

    @Override
    public boolean isFinished() {
        return finalized;
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            chassis.align();
        } else {
            chassis.updateOffsets();
        }
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
