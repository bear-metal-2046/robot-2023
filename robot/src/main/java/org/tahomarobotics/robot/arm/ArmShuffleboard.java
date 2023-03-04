package org.tahomarobotics.robot.arm;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import org.tahomarobotics.robot.util.Shufflebear;

import java.util.Map;

public class ArmShuffleboard {

    private final GenericEntry finalize;
    private final GenericEntry shoulderEntry;
    private final GenericEntry elbowEntry;
    private final GenericEntry shoulderVoltageEntry;
    private final GenericEntry elbowVoltageEntry;
    private final GenericEntry shoulderCurrentEntry;
    private final GenericEntry elbowCurrentEntry;
    private final GenericEntry armXEntry;
    private final GenericEntry armYEntry;
    private final Arm arm;

    public ArmShuffleboard(Arm arm) {
        this.arm = arm;

        ShuffleboardLayout armInfoLayout = Shufflebear.makeLayout("Arm", "Arm Info", 3, 0, 1, 4, BuiltInLayouts.kList);
        Shufflebear.makeLayout("Arm", "Calibration", 1, 0, 2, 2, BuiltInLayouts.kGrid, Map.of("Label position", "HIDDEN"));

        Shufflebear.addSendable("Arm", "Arm Instance", (Sendable) arm, 0, 0, 1, 2);

        shoulderEntry = Shufflebear.addNumber("Arm", "Shoulder Angle", 0, 0, 2, 2, 1).getEntry();
        elbowEntry = Shufflebear.addNumber("Arm", "Elbow Angle", 0, 0, 3, 2, 1).getEntry();

        shoulderVoltageEntry = Shufflebear.addNumber("Arm", "Shoulder Voltage", 0, 0, 0, 1, 1, "Arm Info").getEntry();
        elbowVoltageEntry = Shufflebear.addNumber("Arm", "Elbow Voltage", 0, 0, 1, 1, 1, "Arm Info").getEntry();
        shoulderCurrentEntry = Shufflebear.addNumber("Arm", "Shoulder Current", 0, 0, 0, 1, 1, "Arm Info").getEntry();
        elbowCurrentEntry = Shufflebear.addNumber("Arm", "Elbow Current", 0, 0, 1, 1, 1, "Arm Info").getEntry();

        armXEntry = Shufflebear.addNumber("Arm", "Arm X", 0, 4, 0, 1, 1).getEntry();
        armYEntry = Shufflebear.addNumber("Arm", "Arm Y", 0, 4, 1, 1, 1).getEntry();


        finalize = Shufflebear.addBool("Arm", "Finalize?", false, 2, 2, 1, 1, "Calibration").getEntry();

        ArmCalibrationCommand calibrationCommand = new ArmCalibrationCommand(arm,
                () -> finalize.getBoolean(false),
                finalize::setBoolean);

        Shufflebear.addSendable("Arm", "Arm Calibration", calibrationCommand,0, 0, 3, 1, "Calibration");

    }

    void update() {
        ArmState state = arm.getCurrentArmState();
        shoulderEntry.setDouble(Units.radiansToDegrees(state.shoulder.position()));
        elbowEntry.setDouble(Units.radiansToDegrees(state.elbow.position()));

        Translation2d position = arm.getCurrentPosition();
        armXEntry.setDouble(Units.metersToInches(position.getX()));
        armYEntry.setDouble(Units.metersToInches(position.getY()));

        Arm.ArmElectricalInfo electricalInfo = arm.getArmElectricalInfo();
        shoulderVoltageEntry.setDouble(electricalInfo.shoulderVoltage());
        elbowVoltageEntry.setDouble(electricalInfo.elbowVoltage());
        shoulderCurrentEntry.setDouble(electricalInfo.shoulderCurrent());
        elbowCurrentEntry.setDouble(electricalInfo.elbowCurrent());

    }
}
