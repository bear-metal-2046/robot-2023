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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.SubsystemIF;
import org.tahomarobotics.robot.util.SparkMaxHelper;

public class Grabber extends SubsystemBase implements SubsystemIF {
    private static final Logger logger = LoggerFactory.getLogger(Grabber.class);
    private static final Grabber INSTANCE = new Grabber();

    private final CANSparkMax grabberMotor = new CANSparkMax(RobotMap.GRABBER_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final RelativeEncoder encoder;
    private Grabber() {

        encoder = grabberMotor.getEncoder();
        SparkMaxHelper.checkThenConfigure("Grabber Motor", logger, GrabberConstants.createMotorConfig(), grabberMotor, encoder);
    }
    public static Grabber getInstance() {
        return INSTANCE;
    }


    public void setGrabberSpeed(double percentage) {
       grabberMotor.set(percentage);
    }

    public double getVelocity() {
        return encoder.getVelocity();
    }
}
