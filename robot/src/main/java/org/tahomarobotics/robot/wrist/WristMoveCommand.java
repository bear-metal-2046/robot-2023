package org.tahomarobotics.robot.wrist;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.tahomarobotics.robot.motion.MotionProfile;
import org.tahomarobotics.robot.motion.MotionState;
import org.tahomarobotics.robot.motion.SCurveMotionProfile;


public class WristMoveCommand extends CommandBase {

    public enum WristPositions {
        //TODO measure these
        STOW(Units.degreesToRadians(121d)),
        DOWN_COLLECT(Units.degreesToRadians(102d)),
        UP_COLLECT(Units.degreesToRadians(95.82)),
        GROUNDPLACE(Units.degreesToRadians(0)),
        MIDBOXPLACE(Units.degreesToRadians(-38d)),
        MIDPOLEPLACE(Units.degreesToRadians(-60)),
        HIGHPOLEPLACE(Units.degreesToRadians(-45)),
        PLACE(Units.degreesToRadians(-70d)),
        SCORE(Units.degreesToRadians(-64));

        public double angle;
        WristPositions(double angle) {
            this.angle = angle;
        }
    }


    Wrist wrist = Wrist.getInstance();
    Timer timer;
    private SCurveMotionProfile motionProf;
    private final double angle;
    private final double time;
    private MotionState motionState = new MotionState();

    public WristMoveCommand(double angle, double time) {
        this.angle = angle;
        this.time = time;
        timer = new Timer();
        addRequirements(wrist);
    }
    public WristMoveCommand() {
        this.angle = 0d;
        this.time = 0d;
        motionProf = null;
        timer = new Timer();
        addRequirements(wrist);
    }

    private SCurveMotionProfile generateMotion(double startAngle, double endAngle, double time) throws MotionProfile.MotionProfileException {
        SCurveMotionProfile prof;
        double distance = endAngle - startAngle;
        double velocity = distance/time;

        prof = new SCurveMotionProfile(0, startAngle, endAngle, 0, 0, velocity, WristConstants.MAX_ACCEL, 0.1);

        return prof;
    }


    @Override
    public void initialize() {
        try {
            motionProf = generateMotion(wrist.getPosition(), angle, time);
        } catch (MotionProfile.MotionProfileException e) {
            throw new RuntimeException(e);
        }
        timer.start();
    }

    @Override
    public void execute() {
        if (motionProf != null) {
            motionProf.getSetpoint(timer.get(), motionState);
            if (motionState != null) {
                wrist.goToAngle(motionState.position);
                SmartDashboard.putNumber("Wrist Movement Time", timer.get());
                SmartDashboard.putNumber("Wrist Desired Position", motionState.position);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        boolean atPos = Math.abs(angle - wrist.getPosition()) < 0.05;
        return atPos || motionProf == null;
    }
}
