package org.tahomarobotics.robot.auto;

import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import org.tahomarobotics.robot.chassis.Chassis;

import java.util.HashMap;
import java.util.Map;

public class AutoShuffleboard {
    ShuffleboardTab tab;
    ComplexWidget autonomousChooser;
    ComplexWidget fieldView;
    public AutoShuffleboard(SendableChooser<Command> chooser) {

        tab = Shuffleboard.getTab("Auto");

        autonomousChooser = tab.add("Auto Chooser", chooser)
                .withPosition(0, 0)
                .withSize(2, 3);

        fieldView = tab.add(Chassis.getInstance().getFieldView())
                .withPosition(2, 0)
                .withSize(5, 3)
                .withProperties(new HashMap<>(){{
                    put("traj0", "red");
                    put("traj1", "green");
                    put("traj2", "yellow");
                    put("traj3", "blue");
                }});
    }
}
