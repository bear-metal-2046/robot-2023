package org.tahomarobotics.robot.climb;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import org.tahomarobotics.robot.climb.Paw;
import org.tahomarobotics.robot.util.Shufflebear;

import java.util.HashMap;

public class PawShuffleboard {
    private GenericEntry pawEntry;
    private Paw paw;
    private static ComplexWidget leftInstance = Shufflebear.addSendable("Paw", Paw.getLeftInstance().getName() + " Instance", Paw.getLeftInstance(), 2, 0, 1, 2);
    private static ComplexWidget rightInstance = Shufflebear.addSendable("Paw", Paw.getRightInstance().getName() + " Instance", Paw.getRightInstance(), 4, 0, 1, 2);
    public PawShuffleboard(String name, int x, Paw paw) {
        this.paw = paw;
        pawEntry = Shufflebear.addNumber("Paw", name + "Pos", 0, x, 0, 1, 1)
                .withWidget(BuiltInWidgets.kDial)
                .withProperties(new HashMap<>() {{
                    put("Max", 180d);
                    put("Min", 0d);}}
                ).getEntry();
    }

    public void setPawPos(double value) {
        pawEntry.setDouble(value);
    }
    public void update() {
        pawEntry.setDouble(Units.radiansToDegrees(paw.getPos()));
    }
}
