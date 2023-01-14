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
package org.tahomarobotics.robot.chassis;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

/**
 * Basic Teleoperated Drive Command
 * Used to make the robot go vrmmm
 */

public class TeleopDriveCommand extends CommandBase {
    private final DoubleSupplier xSup, ySup, rotSup;

    private final Chassis chassis = Chassis.getInstance();

    public TeleopDriveCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rotation) {
        this.xSup = x;
        this.ySup = y;
        this.rotSup = rotation;

        addRequirements(chassis);
    }

    @Override
    public void execute() {
        chassis.drive(
            xSup.getAsDouble(),
            ySup.getAsDouble(),
            rotSup.getAsDouble()
        );
    }

    @Override
    public void end(boolean interrupted) {
        chassis.drive(0.0, 0.0, 0.0);
    }

}
