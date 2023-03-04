package org.tahomarobotics.robot.auto;

import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import org.tahomarobotics.robot.util.Shufflebear;

public class AutoShuffleboard {
    public static ComplexWidget autoChooser = Shufflebear.addSendable("Main", "Auto Chooser", Autonomous.getInstance().getAutoPathChooser(), 0, 0, 2, 3);

}
