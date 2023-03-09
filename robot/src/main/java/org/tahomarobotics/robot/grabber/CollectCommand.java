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
package org.tahomarobotics.robot.grabber;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import static org.tahomarobotics.robot.grabber.GrabberConstants.*;

public class CollectCommand extends CommandBase {

    private final DoubleSupplier driveLeftTrigger;
    private final DoubleSupplier manipLeftTrigger;
    private final BooleanSupplier ejectButton;

    Grabber grabber = Grabber.getInstance();
    public CollectCommand(DoubleSupplier driveLeftTrigger, DoubleSupplier manipLeftTrigger, BooleanSupplier ejectButton) {
        this.driveLeftTrigger = driveLeftTrigger;
        this.manipLeftTrigger = manipLeftTrigger;
        this.ejectButton = ejectButton;
        addRequirements(grabber);
    }

    @Override
    public void execute() {
        double ingest = driveLeftTrigger.getAsDouble();
        double score = manipLeftTrigger.getAsDouble();
        boolean isEject = ejectButton.getAsBoolean();
        boolean isScore = score > TRIGGER_DEAD_ZONE;
        boolean isIngest = ingest > TRIGGER_DEAD_ZONE;

        switch (grabber.getState()) {
            case OFF, RETAIN -> {

                if (isIngest)  grabber.ingest(ingest);

                else if (isScore) grabber.score(score);

                else if (isEject) grabber.eject();
            }

            case INGEST -> {

                grabber.ingest(ingest);

                if ( ! isIngest) grabber.retain();

                else if (isEject) grabber.eject();
            }

            case SCORE -> {

                grabber.score(score);

                if ( ! isScore) grabber.off();
            }

            case EJECT -> {

                if ( ! isEject ) grabber.off();
            }
        }
    }
}
