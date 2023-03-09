package org.tahomarobotics.robot.arm;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import org.tahomarobotics.robot.OI;

import java.util.Map;

import static org.tahomarobotics.robot.OperatorArmMoveSelection.*;

public class ArmShuffleboard {

    private final GenericEntry shoulderAngle;
    private final GenericEntry elbowAngle;
    private final GenericEntry xPosition;
    private final GenericEntry yPosition;

    private final ShuffleboardTab tab;
    private final GenericEntry shoulderVoltage;
    private final GenericEntry shoulderCurrent;
    private final GenericEntry elbowVoltage;
    private final GenericEntry elbowCurrent;
    private GenericEntry gamePieceMode = null;

    private final Arm arm;

    public ArmShuffleboard(Arm arm) {
        this.arm = arm;

        tab = Shuffleboard.getTab("Arm");

        // Display Angles and Positions
        var positionLayout = tab.getLayout("Arm Positions", BuiltInLayouts.kGrid)
                .withPosition(4,0)
                .withSize(4, 4)
                .withProperties(Map.of(
                        "Number of columns", 2,
                        "Number of rows", 2,
                        "Label position", "BOTTOM"));

        shoulderAngle = positionLayout.add("Shoulder Angle", 0)
                .withPosition(0,0)
                .withWidget(BuiltInWidgets.kDial).withProperties(Map.of(
                        "min", -90.0,
                        "max", 90.0
                )).getEntry();

        elbowAngle = positionLayout.add("Elbow Angle", 0)
                .withPosition(1,0)
                .withWidget(BuiltInWidgets.kDial).withProperties(Map.of(
                        "min", -180.0,
                        "max", 0.0
                )).getEntry();

        xPosition = positionLayout.add("X Position", 0)
                .withPosition(0,1)
                .withWidget(BuiltInWidgets.kNumberBar)
                .withProperties(Map.of(
                        "min", -60.0,
                        "max", 60.0,
                        "orientation", "HORIZONTAL",
                        "Num tick marks", 5
                ))
                .getEntry();

        yPosition = positionLayout.add("Y Position", 0)
                .withPosition(1,1)
                .withWidget(BuiltInWidgets.kNumberBar)
                .withProperties(Map.of(
                        "min", -20.0,
                        "max", 60.0,
                        "orientation", "VERTICAL",
                        "Num tick marks", 5
                ))
                .getEntry();

        // Display Voltages and Currents
        var electrical = tab.getLayout("Arm Electrical", BuiltInLayouts.kGrid)
                .withPosition(0,0)
                .withSize(4, 4)
                .withProperties(Map.of(
                        "Number of columns", 1,
                        "Number of rows", 4,
                        "Label position", "BOTTOM"));

        shoulderVoltage = electrical.add("Shoulder Voltage", 0.0)
                .withPosition(0,0)
                .withWidget(BuiltInWidgets.kVoltageView)
                .withProperties(Map.of("min", -15, "max", 15))
                .getEntry();
        shoulderCurrent = electrical.add("Shoulder Current", 0.0)
                .withPosition(0,1)
                .withWidget(BuiltInWidgets.kNumberBar)
                .withProperties(Map.of("min", 0, "max", 60))
                .getEntry();
        elbowVoltage = electrical.add("Elbow Voltage", 0.0)
                .withPosition(0,2)
                .withWidget(BuiltInWidgets.kVoltageView)
                .withProperties(Map.of("min", -15, "max", 15))
                .getEntry();
        elbowCurrent = electrical.add("Elbow Current", 0.0)
                .withPosition(0,3)
                .withWidget(BuiltInWidgets.kNumberBar)
                .withProperties(Map.of("min", 0, "max", 60))
                .getEntry();


        var armCalibrationControls = tab.getLayout("Arm Calibration Controls", BuiltInLayouts.kGrid)
                .withSize(2, 1).withPosition(10,3)
                .withProperties(Map.of(
                        "Number of columns", 1,
                        "Number of rows", 2,
                        "Label position", "HIDDEN"));

        var finalize = armCalibrationControls.add("Finalize ArmCal", Boolean.FALSE)
                .withPosition(0,1)
                .withWidget(BuiltInWidgets.kToggleButton)
                .getEntry();

        var cmd = new ArmCalibrationCommand(arm, () -> finalize.getBoolean(false), finalize::setBoolean);
        cmd.setName("Arm Calibrate");
        armCalibrationControls.add(cmd);

    }

    void update() {
        ArmState state = arm.getCurrentArmState();
        shoulderAngle.setDouble(Units.radiansToDegrees(state.shoulder.position()));
        elbowAngle.setDouble(Units.radiansToDegrees(state.elbow.position()));

        Translation2d position = arm.getCurrentPosition();
        xPosition.setDouble(Units.metersToInches(position.getX()));
        yPosition.setDouble(Units.metersToInches(position.getY()));

        Arm.ArmElectricalInfo electricalInfo = arm.getArmElectricalInfo();
        shoulderVoltage.setDouble(electricalInfo.shoulderVoltage());
        elbowVoltage.setDouble(electricalInfo.elbowVoltage());
        shoulderCurrent.setDouble(electricalInfo.shoulderCurrent());
        elbowCurrent.setDouble(electricalInfo.elbowCurrent());


        if (gamePieceMode != null) {
            var mode = OI.getInstance().getArmMoveSelector().getGamePieceMode();
            gamePieceMode.setBoolean(mode == ConeOrCube.CONE);
        }
    }

    public void initialize() {

        var selector = OI.getInstance().getArmMoveSelector();

        var armControls = tab.getLayout("Arm Move Controls", BuiltInLayouts.kGrid)
                .withSize(4, 2).withPosition(8,0)
                .withProperties(Map.of(
                        "Number of columns", 1,
                        "Number of rows", 3,
                        "Label position", "HIDDEN"));

        var armScoreControls = armControls.getLayout("score-cmd", BuiltInLayouts.kGrid)
                .withProperties(Map.of(
                        "Number of columns", 3,
                        "Number of rows", 1,
                        "Label position", "HIDDEN"));

        armScoreControls.add("Score-High", selector.setScoringLevel(ScoringLevel.HIGH));
        armScoreControls.add("Score-Mid", selector.setScoringLevel(ScoringLevel.MID));
        armScoreControls.add("Score!", selector.toggleScoring());

        var armCollectControls = armControls.getLayout("collect-cmd", BuiltInLayouts.kGrid)
                .withProperties(Map.of(
                        "Number of columns", 2,
                        "Number of rows", 1,
                        "Label position", "HIDDEN"));

        armCollectControls.add("Collect-FEEDER", selector.toggleCollecting(CollectLevel.FEEDER));
        armCollectControls.add("Collect-LOW", selector.toggleCollecting(CollectLevel.LOW));

        var armGamePieceMode = armControls.getLayout("game-piece-mode", BuiltInLayouts.kGrid)
                .withProperties(Map.of(
                        "Number of columns", 4,
                        "Number of rows", 1,
                        "Label position", "HIDDEN"));

        armGamePieceMode.add("Toggle Game-Piece", selector.toggleGamePieceMode())
                .withPosition(0,0);

        gamePieceMode = armGamePieceMode.add("game-piece-mode-value", Boolean.FALSE)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withPosition(1, 0)
                .withProperties(Map.of("Color when false", "MAGENTA", "Color when true", "YELLOW"))
                .getEntry();

        armGamePieceMode.add("Cube", selector.gamePieceMode(ConeOrCube.CUBE))
                .withPosition(2,0);
        armGamePieceMode.add("Cone", selector.gamePieceMode(ConeOrCube.CONE))
                .withPosition(3,0);








    }

}
