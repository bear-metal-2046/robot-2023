/**
 * Copyright 2023 Tahoma Robotics - http://tahomarobotics.org - Bear Metal 2046 FRC Team
 * <p>
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including without
 * limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
 * Software, and to permit persons to whom the Software is furnished to do so, subject to the following
 * conditions:
 * <p>
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions
 * of the Software.
 * <p>
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
 * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */
package org.tahomarobotics.robot.wrist;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.tahomarobotics.robot.Robot;
import org.tahomarobotics.robot.SubsystemIF;
import org.tahomarobotics.robot.util.CalibrationAction;

public class Wrist extends SubsystemBase implements SubsystemIF {
    private static final Wrist INSTANCE = new Wrist(Robot.isReal() ? new WristIO.WristIOPhys() : new WristIO.WristIOSim());

    public static Wrist getInstance() {
        return INSTANCE;
    }

    private final WristIO io;
    private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

    private Wrist(WristIO io) {
        this.io = io;
    }

    public Wrist initialize() {
        SmartDashboard.putData("Wrist Calibration", new WristCalibrationCommand());
        io.initialize();
        return this;
    }

    public void setPosition(double angle) {
        io.setPosition(angle);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Wrist", inputs);
        if (DriverStation.isDisabled())
            io.disable();
    }

    public void calibration(CalibrationAction calibrationAction) {
        io.calibration(calibrationAction);
    }

    public double getPosition() {
        return io.getPosition();
    }

    public double getVelocity() {
        return io.getVelocity();
    }
}


