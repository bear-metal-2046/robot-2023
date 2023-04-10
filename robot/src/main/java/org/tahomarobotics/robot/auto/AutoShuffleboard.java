package org.tahomarobotics.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import org.tahomarobotics.robot.chassis.Chassis;

import java.util.HashMap;

public class AutoShuffleboard {
    ShuffleboardTab tab;
    ComplexWidget autonomousChooser;
    ComplexWidget fieldView;
    GenericEntry poseX, poseY, poseRot;

    public AutoShuffleboard(SendableChooser<Autonomous.AutonomousOption> chooser) {

        tab = Shuffleboard.getTab("Auto");

        autonomousChooser = tab.add("Auto Chooser", chooser)
                .withPosition(0, 0)
                .withSize(2, 1);

        fieldView = tab.add(Chassis.getInstance().getFieldView())
                .withPosition(2, 0)
                .withSize(5, 3)
                .withProperties(new HashMap<>(){{
                    put("traj0", "red");
                    put("traj1", "green");
                    put("traj2", "yellow");
                    put("traj3", "blue");
                }});


        poseX = tab.add("X", 0d)
                .withPosition(0, 1)
                .withSize(2, 1)
                .getEntry();

        poseY = tab.add("Y", 0d)
                .withPosition(0, 2)
                .withSize(2, 1)
                .getEntry();

        poseRot = tab.add("ROTATION", 0d)
                .withPosition(0, 3)
                .withSize(2, 1)
                .getEntry();
    }

    void update() {
        Pose2d pos = Chassis.getInstance().getPose();
        poseX.setDouble(Units.metersToInches(pos.getX()));
        poseY.setDouble(Units.metersToInches(pos.getY()));
        poseRot.setDouble(pos.getRotation().getDegrees());
    }
}
