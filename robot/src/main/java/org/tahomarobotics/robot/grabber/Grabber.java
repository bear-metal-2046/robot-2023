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

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.OI;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.SubsystemIF;
import org.tahomarobotics.robot.util.SparkMaxHelper;

import static org.tahomarobotics.robot.grabber.GrabberConstants.EJECT_SPEED;
import static org.tahomarobotics.robot.grabber.GrabberConstants.RETAIN_SPEED;

public class Grabber extends SubsystemBase implements SubsystemIF {
    private static final Logger logger = LoggerFactory.getLogger(Grabber.class);
    private static final Grabber INSTANCE = new Grabber();

    private final CANSparkMax grabberMotor;
    private final RelativeEncoder encoder;

    protected enum MovementState {  OFF, INGEST, RETAIN, EJECT, SCORE  }

    private MovementState state = MovementState.OFF;

    private final Timer injectStallTimer = new Timer();

    private boolean retained = false;

    private Grabber() {
        grabberMotor = new CANSparkMax(RobotMap.GRABBER_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        encoder = grabberMotor.getEncoder();
        SparkMaxHelper.checkThenConfigure("Grabber Motor", logger, GrabberConstants.createMotorConfig(), grabberMotor, encoder);
    }

    MovementState getState() {
        return state;
    }
    /**
     * THIS CONSTRUCTOR FOR TEST PURPOSES ONLY!
     *
     * @param testOnly - not used
     */
    protected Grabber(@SuppressWarnings("unused") boolean testOnly) {
        grabberMotor = null;
        encoder = null;
    }

    public static Grabber getInstance() {
        return INSTANCE;
    }


    void setGrabberSpeed(double percentage) {
        grabberMotor.set(percentage);
    }

    double getCurrent() {
        return grabberMotor.getOutputCurrent();
    }

    double getPower() {
        return OI.getInstance().getDriverLeftAxis();
    }


    void off() {
        retained = false;
        state = MovementState.OFF;
        setGrabberSpeed(0);
    }

    public void ingest(double speed) {

        state = MovementState.INGEST;

        // clear timer if not stalled
        if (!isStalled()) {
            injectStallTimer.restart();
        }

        if (retained || injectStallTimer.hasElapsed(GrabberConstants.INTAKE_TIMOUT)) {
            setGrabberSpeed(RETAIN_SPEED);
            retained = true;
        } else {
            setGrabberSpeed(Math.max(RETAIN_SPEED, speed));
        }
    }

    public void retain() {
        state = MovementState.RETAIN;
        setGrabberSpeed(RETAIN_SPEED);
    }

    public void score(double level) {
        state = MovementState.SCORE;
        setGrabberSpeed(-level);
    }

    public void eject() {
        state = MovementState.EJECT;
        setGrabberSpeed(EJECT_SPEED);
    }

    public boolean isStalled() {
        double current = getCurrent();
        SmartDashboard.putNumber("Grabber Current", current);
        return current > 60 + 10 * getPower();
    }
}