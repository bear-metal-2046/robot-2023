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

public class PawCommand extends CommandBase {
    private static final Logger logger = LoggerFactory.getLogger(PawCommand.class);
    private Timer timer;
    private final Paw paw;
    private MotionProfile motionProfile;
    private final MotionState motionState = new MotionState();
    private final double desiredPos;
    private final double maxVelocity;
    private final double accelFactor = 0.25;
    private final double jerkFactor = 0.05;
    private boolean failedSetup = false;
    private double commandEndTime;
    private final double commandTimeOut = 0.5;
    private final ChartData velocityData;
    private final ChartData angleData;

    public PawCommand(Paw paw, double desiredPos, double maxVelocity) {
        this.paw = paw;
        this.desiredPos = desiredPos;
        this.maxVelocity = maxVelocity;

        velocityData = new ChartData(paw.getName() + " Velocity", "Time", "Velocity", new String[]{
                "Expected Velocity",
                "Actual Velocity"
        });
        angleData = new ChartData(paw.getName() + " Angle", "Time", "Degrees", new String[]{
                "Expected Angle",
                "Actual Angle"
        });
    }

    @Override
    public void initialize() {

        timer = new Timer();
        timer.start();

        velocityData.clear();
        angleData.clear();

        double maxAcceleration = maxVelocity / accelFactor;
        double maxJerk = maxAcceleration / jerkFactor;
        try {
            motionProfile = new SCurveMotionProfile(0, paw.getPos(), desiredPos, 0,
                    0, maxVelocity, maxAcceleration, maxJerk);
            commandEndTime = motionProfile.getEndTime() + commandTimeOut;
        } catch (MotionProfile.MotionProfileException e) {
            logger.error("Failed setting up motion profile", e);
            failedSetup = true;
        }

    }

    @Override
    public void execute() {
        double time = timer.get();
        motionProfile.getSetpoint(time, motionState);
        paw.setGoal(motionState);

        //Chart data stuff
        double[] velocities = {time,
                motionState.velocity,
                paw.getVelocity()
        };

        velocityData.addData(velocities);

        double[] angles = {time,
                Units.radiansToDegrees(motionState.position),
                Units.radiansToDegrees(paw.getPos())
        };

        angleData.addData(angles);
    }

    @Override
    public boolean isFinished() {
        if (failedSetup) return true;
        return (timer.hasElapsed(commandEndTime));
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putRaw("Paw Velocities", velocityData.serialize());
        SmartDashboard.putRaw("Paw Degrees", angleData.serialize());
        logger.info(paw.getName() + " command to position "+ desiredPos +" complete");
    }
}
