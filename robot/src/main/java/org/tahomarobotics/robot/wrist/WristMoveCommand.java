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
package org.tahomarobotics.robot.wrist;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.motion.MotionProfile;
import org.tahomarobotics.robot.motion.MotionState;
import org.tahomarobotics.robot.motion.SCurveMotionProfile;
import org.tahomarobotics.robot.util.ChartData;


public class WristMoveCommand extends CommandBase {
    private static final Logger logger = LoggerFactory.getLogger(WristMoveCommand.class);

    private final Wrist wrist = Wrist.getInstance();
    private final Timer timer;
    private SCurveMotionProfile motionProf;
    private final double angle;
    private final double time;
    private final MotionState motionState = new MotionState();

    private final ChartData velocityData;
    private final ChartData angleData;
    public WristMoveCommand(WristPosition position, double time) {
        this.angle = position.angle;
        this.time = time;
        timer = new Timer();
        addRequirements(wrist);

        velocityData = new ChartData(" Velocity", "Time", "Velocity", new String[]{
                "Expected Velocity",
                "Actual Velocity"
        });
        angleData = new ChartData(" Angle", "Time", "Degrees", new String[]{
                "Expected Angle",
                "Actual Angle"
        });
    }

    private SCurveMotionProfile generateMotion(double startTime, double startAngle, double endAngle, double time) {
        SCurveMotionProfile prof = null;
        double distance = Math.abs(endAngle - startAngle);
        double tj = time * 0.04;
        double ta = time * 0.2;
        double tv = time - 2 * ta - 4 * tj;
        double maxVel = distance/(tv + ta + tj);
        double maxAcc = maxVel/(ta + tj);
        double maxJerk = maxAcc/tj;

        try {
            prof = new SCurveMotionProfile(startTime, startAngle, endAngle, 0, 0, maxVel, maxAcc, maxJerk);
        } catch (MotionProfile.MotionProfileException e) {
            logger.error("Failed to create S-Curve profile", e);
        }

        return prof;
    }

    @Override
    public void initialize() {
        velocityData.clear();
        angleData.clear();
        timer.restart();
        motionProf = generateMotion(timer.get(), wrist.getPosition(), angle, time);
    }

    @Override
    public void execute() {
        double currentTime = timer.get();
        if (motionProf != null) {
            motionProf.getSetpoint(currentTime, motionState);
            wrist.setPosition(motionState.position);
        }

        double[] velData = {currentTime,
            motionState.velocity,
            wrist.getVelocity()
        };

        velocityData.addData(velData);

        double[] posData = {currentTime,
            Units.radiansToDegrees(motionState.position),
            Units.radiansToDegrees(wrist.getPosition())
        };

        angleData.addData(posData);
    }

    @Override
    public void end(boolean interrupted) {
        if (motionProf != null) {
            wrist.setPosition(motionProf.getEndPosition());
        }
        timer.stop();
        SmartDashboard.putRaw("Wrist Velocity", velocityData.serialize());
        SmartDashboard.putRaw("Wrist Degrees", angleData.serialize());
    }

    @Override
    public boolean isFinished() {
        return motionProf == null || timer.hasElapsed(motionProf.getEndTime());
    }
}
