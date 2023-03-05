package org.tahomarobotics.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.tahomarobotics.robot.chassis.Chassis;

public class NoOperation extends SequentialCommandGroup implements AutonomousCommandIF {
    @Override
    public void onSelection() {
        Chassis.getInstance().updateTrajectory(null);
    }
}
