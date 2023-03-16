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
package org.tahomarobotics.robot.grabber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.OI;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static org.tahomarobotics.robot.grabber.GrabberConstants.TRIGGER_DEAD_ZONE;

public class CollectCommand extends CommandBase {
    private static final Logger logger = LoggerFactory.getLogger(CollectCommand.class);
    private final DoubleSupplier driveLeftTrigger;
    private final DoubleSupplier manipLeftTrigger;
    private final BooleanSupplier ejectButton;

    private Grabber grabber = Grabber.getInstance();

    private boolean isRumbled = false;

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

                isRumbled = false;

                if (isIngest) grabber.ingest(ingest);

                else if (isScore) grabber.score(score);

                else if (isEject) grabber.eject();
            }

            case INGEST -> {

                grabber.ingest(ingest);

                if  (!isRumbled && grabber.isStalled()) {
                    logger.info("Rumble");
                    OI.getInstance().rumbleDrive();
                    isRumbled = true;
                }

                if (!isIngest) {
                    grabber.retain();
                }
                else if (isEject) grabber.eject();
            }

            case SCORE -> {

                grabber.score(score);

                if (!isScore) grabber.off();
            }

            case EJECT -> {

                isRumbled = false;

                if (!isEject) grabber.off();
            }
        }
    }
}
