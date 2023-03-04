package org.tahomarobotics.robot.grabber;

import edu.wpi.first.networktables.GenericEntry;
import org.tahomarobotics.robot.util.Shufflebear;

public class GrabberShuffleboard {
    GenericEntry stateEntry = Shufflebear.addString("Grabber", "Current State", "Off", 2, 0, 1, 1).getEntry();
    public GrabberShuffleboard() {
        Shufflebear.addSendable("Grabber", "Grabber Instance", Grabber.getInstance(), 0, 0, 2, 2);
    }
    public void update(String state) {
        stateEntry.setString(state);
    }
}
