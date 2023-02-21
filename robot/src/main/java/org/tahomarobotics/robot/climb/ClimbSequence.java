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
package org.tahomarobotics.robot.climb;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.tahomarobotics.robot.chassis.DriveForwardCommand;

public class ClimbSequence extends SequentialCommandGroup {
    private final Paw left = Paw.getLeftInstance();
    private final Paw right = Paw.getRightInstance();
    private static final double START_MOVE_ANGLE = 1.9; // radians
    private static final double FINAL_ANGLE = 2.72;

    private static final double VELOCITY = 2;
    public ClimbSequence() {

        addCommands(
                new ParallelCommandGroup(
                        new PawCommand(left, FINAL_ANGLE, VELOCITY),

                        new PawCommand(right, FINAL_ANGLE, VELOCITY),
                        Commands.waitUntil(
                                () -> left.getPos() > START_MOVE_ANGLE && right.getPos() > START_MOVE_ANGLE)
                                .andThen(new DriveForwardCommand(-0.75, 2.5)
                                .alongWith(new BeacherCommand(2.5, 0.25)))
                )
        );
    }
}
