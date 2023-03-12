package org.tahomarobotics.robot.climb;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class BeacherCommand extends CommandBase {
    private static final Logger logger = LoggerFactory.getLogger(BeacherCommand.class);
    private Beacher beacher = Beacher.getBeacherInstance();
    private final Timer timer = new Timer();
    private final double time;
    private final double speed;
    public BeacherCommand(double time, double speed) {
        this.time = time;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        beacher.runBeacher(speed);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(time);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        beacher.stopBeacher();
    }
}
