package org.tahomarobotics.robot.auto;

import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.SubsystemIF;
import org.tahomarobotics.robot.arm.ArmMovements;

import java.util.EnumSet;
import java.util.HashMap;
import java.util.Map;

public class Autonomous extends SubsystemBase implements SubsystemIF {
    /** Initialize this classes logger. */
    private static final Logger logger = LoggerFactory.getLogger(Autonomous.class);

    private static final Autonomous INSTANCE = new Autonomous();

    public static Autonomous getInstance(){
        return INSTANCE;
    }

    private final Map<String, Command> autoCommands = new HashMap<>();
    private final SendableChooser<Command> autoCommandChooser = new SendableChooser<>();
    private final Command defaultCommand = new NoOperation(DriverStation.Alliance.Blue);
    private Command autonomousCommand;
    private AutoShuffleboard shuffleboard;


    public Autonomous initialize(){
        shuffleboard = new AutoShuffleboard(autoCommandChooser);

        addAuto(defaultCommand, new NoOperation(DriverStation.Alliance.Red));

        addAuto(new PlaceTaxi(),
                new PlaceTaxi());

        addAuto(new MidPlaceEngage(),
                new MidPlaceEngage());

        addAuto(new PlaceCollectPlace(DriverStation.Alliance.Blue),
                new PlaceCollectPlace(DriverStation.Alliance.Red));

                addAuto(new PlaceCollect(DriverStation.Alliance.Blue),
                new PlaceCollect(DriverStation.Alliance.Red));

        SmartDashboard.putData("AutonomousChooser", autoCommandChooser);
        selectionAutoChange(autoCommandChooser.getSelected());


        NetworkTableInstance netInstance = NetworkTableInstance.getDefault();
        StringSubscriber subscriber = netInstance.getTable("Shuffleboard").getSubTable("Auto/Auto Chooser").getStringTopic("selected").subscribe("defaultAutoCommand");
        netInstance.addListener(subscriber, EnumSet.of(NetworkTableEvent.Kind.kValueAll), e -> {
            selectionAutoChange(autoCommands.get((String) e.valueData.value.getValue()));
        });

        return this;
    }

    @Override
    public void periodic() {
        shuffleboard.update();
    }

    private Map<Command, Command> blueToRed = new HashMap<>();

    private void addAuto(Command command, Command red) {
        blueToRed.put(command, red);
        autoCommands.put(command.getName(), command);

        if (command == defaultCommand) {
            autoCommandChooser.setDefaultOption(command.getName(), command);
        } else {
            autoCommandChooser.addOption(command.getName(), command);
        }
    }

    /**
     * Listener for "AutonomousChooser/selected" change.
     */
    private void selectionAutoChange(Command command) {
        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
            command = blueToRed.get(command);
        }
        if (command instanceof AutonomousCommandIF) {
            ((AutonomousCommandIF) command).onSelection();
        }
    }

    private Command getSelectedCommand() {
        var command = autoCommandChooser.getSelected();
        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
            command = blueToRed.get(command);
        }
        return command;
    }

    @Override
    public void onDisabledInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void onAutonomousInit() {
        autonomousCommand = getSelectedCommand();
        logger.info("Running " + autonomousCommand.getName() + "...");
        autonomousCommand.schedule();
    }

    @Override
    public void onTeleopInit() {
        if (autonomousCommand != null) {
            ArmMovements.createPositionToStowCommand().schedule();
            autonomousCommand = null;
        }

    }
}
