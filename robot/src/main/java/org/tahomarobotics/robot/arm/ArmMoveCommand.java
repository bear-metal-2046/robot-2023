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

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.util.ChartData;

public class ArmMoveCommand extends CommandBase {

    private static final Logger logger = LoggerFactory.getLogger(ArmMoveCommand.class);
    private boolean canceled = false;

    Timer timer = new Timer();

    ChartData chartData = new ChartData("Electrical", "Time", "Voltage/Current",
            new String[] { "Shoulder Voltage", "Elbow Voltage", "Shoulder Current", "Elbow Current"});
    ChartData angleChart = new ChartData("Angles", "Time", "Degrees",
            new String[] { "Expected Shoulder", "Expected Elbow", "Actual Shoulder", "Actual Elbow"});

    ChartData angluarVelocityChart = new ChartData("Angular Velocity", "Time", "deg/sec",
            new String[] { "Expected Shoulder", "Expected Elbow", "Actual Shoulder", "Actual Elbow"});

    private final ArmSubsystemIF arm = Arm.getInstance();
    private final ArmTrajectory trajectory;

    public ArmMoveCommand(String name, ArmTrajectory trajectory) {
        setName(name);
        this.trajectory = trajectory;
        if (!trajectory.isValid()) {
            canceled = true;
        }
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        chartData.clear();
        angleChart.clear();
        angluarVelocityChart.clear();
    }

    @Override
    public void execute() {
        if (canceled) {
            return;
        }

        double time = timer.get();
        ArmState desiredState = trajectory.sample(time);
        arm.setArmState(desiredState);

        var armState = arm.getCurrentArmState();
        var elec = arm.getArmElectricalInfo();

        double[] data = {timer.get(), elec.shoulderVoltage(), elec.elbowVoltage(), elec.shoulderCurrent(), elec.elbowCurrent()};
        chartData.addData(data);

        double[] speedData = {timer.get(),
                Units.radiansToDegrees(desiredState.shoulder.velocity()),
                Units.radiansToDegrees(desiredState.elbow.velocity()),
                Units.radiansToDegrees(armState.shoulder.velocity()),
                Units.radiansToDegrees(armState.elbow.velocity())};

        angluarVelocityChart.addData(speedData);

        // expected arm position vs real position
        double[] angleData = {timer.get(),
                Units.radiansToDegrees(desiredState.shoulder.position()),
                Units.radiansToDegrees(desiredState.elbow.position()),
                Units.radiansToDegrees(armState.shoulder.position()),
                Units.radiansToDegrees(armState.elbow.position())};

        angleChart.addData(angleData);
    }

    @Override
    public void end(boolean interrupted) {
        if (canceled) return;
        arm.setArmState(trajectory.getFinalState());
        timer.stop();
        SmartDashboard.putRaw("Arm Chart", chartData.serialize());
        SmartDashboard.putRaw("Velocity Chart", angluarVelocityChart.serialize());
        SmartDashboard.putRaw("Angle Chart", angleChart.serialize());
        logger.info("ArmMove [" + getName() + "] complete");
    }

    @Override
    public boolean isFinished() {
        double time = trajectory.getTotalTimeSeconds();
        return canceled
                || (timer.hasElapsed(time) && arm.isAtPosition())
                || timer.hasElapsed(time + 0.5);
    }
}
