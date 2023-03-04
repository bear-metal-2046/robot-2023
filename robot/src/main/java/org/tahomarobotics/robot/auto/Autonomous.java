package org.tahomarobotics.robot.auto;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.SubsystemIF;
import org.tahomarobotics.robot.chassis.Chassis;

import java.util.EnumSet;
import java.util.HashMap;
import java.util.Map;

public class Autonomous implements SubsystemIF {
    /** Initialize this classes logger. */
    private static final Logger logger = LoggerFactory.getLogger(Autonomous.class);

    private static final Autonomous INSTANCE = new Autonomous();

    public static Autonomous getInstance(){
        return INSTANCE;
    }

    private final Map<String, Command> autoCommands = new HashMap<>();
    private final SendableChooser<Command> autoCommandChooser = new SendableChooser<>();

    private Command defaultCommand; // TODO: SET THIS
    private Command autonomousCommand;

    private static final TrajectoryConfig SWERVE_CONFIG = new TrajectoryConfig(2, 5)
            .setKinematics(Chassis.getInstance().getSwerveDriveKinematics());

    public Autonomous initialize(){
        addAuto(defaultCommand);

        SmartDashboard.putData("AutonomousChooser", autoCommandChooser);
        selectionAutoChange(autoCommandChooser.getSelected());

        NetworkTableInstance netInstance = NetworkTableInstance.getDefault();
        StringSubscriber subscriber = netInstance.getTable("SmartDashboard").getSubTable("AutonomousChooser").getStringTopic("selected").subscribe("defaultAutoCommand");
        netInstance.addListener(subscriber, EnumSet.of(NetworkTableEvent.Kind.kValueAll), e -> {
            selectionAutoChange(autoCommands.get((String) e.valueData.value.getValue()));
        });

        return this;
    }

    public void addAuto(Command command) {
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
        if (command instanceof AutonomousCommandIF) {
            ((AutonomousCommandIF) command).onSelection();
        }
    }

    public Command getSelectedCommand() {
        return autoCommandChooser.getSelected();
    }

    public void initiate() {
        autonomousCommand = getSelectedCommand();
        logger.info("Running " + autonomousCommand.getName() + "...");
        autonomousCommand.schedule();
    }

    public void cancel() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }
}
