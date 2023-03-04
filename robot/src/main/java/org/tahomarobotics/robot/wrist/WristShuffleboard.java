package org.tahomarobotics.robot.wrist;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import org.tahomarobotics.robot.util.Shufflebear;

import java.util.Map;

public class WristShuffleboard {
    GenericEntry CANPosEntry = Shufflebear.addNumber("Wrist", "Wrist CANCoder Position", 0, 0, 2, 2, 1).getEntry();
    GenericEntry RelPosEntry = Shufflebear.addNumber("Wrist", "Wrist Relative Encoder Position", 0, 0, 3, 2, 1).getEntry();
    GenericEntry FINALIZE;
    public WristShuffleboard(){
        Shufflebear.addSendable("Wrist", "Wrist Instance", Wrist.getInstance(), 0, 0, 2, 2);
        Shufflebear.makeLayout("Wrist", "Calibration", 2, 0, 2, 2, BuiltInLayouts.kGrid, Map.of("Label position", "HIDDEN"));
        FINALIZE = Shufflebear.addBool("Wrist", "Finalize?", false, 2, 2, 1, 1, "Calibration").getEntry();
    }

    public void update() {
        CANPosEntry.setDouble(Units.radiansToDegrees(Wrist.getInstance().getCanPosition()));
        RelPosEntry.setDouble(Units.radiansToDegrees(Wrist.getInstance().getRelPosition())) ;
    }

    public boolean getFinalized() {
        return FINALIZE.getBoolean(false);
    }

    public void setFinalized(boolean fin) {
        FINALIZE.setBoolean(fin);
    }

}
