package org.tahomarobotics.robot.chassis;

import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import org.tahomarobotics.robot.util.Shufflebear;

import java.util.HashMap;

public class ChassisShuffleboard {
    public ComplexWidget fieldPoseEntry = Shufflebear.addSendable("Main", "Field Pose", Chassis.getInstance().getFieldPose(), 2, 0, 5, 3).withProperties(new HashMap<>(){{
        put("traj0", "Red");
        put("traj1", "Red");
        put("traj2", "Red");
        put("traj3", "Red");
    }});
    public ChassisShuffleboard() {}
}
