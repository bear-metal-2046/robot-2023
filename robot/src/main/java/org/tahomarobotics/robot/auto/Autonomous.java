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

import java.util.Arrays;
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

    private final Map<String, Path> autoCommands = new HashMap<>();
    private final SendableChooser<Path> autoPathChooser = new SendableChooser<>();

    private Path defaultPath;
    private AutoCommand selectedPathCommand;

    private static final TrajectoryConfig SWERVE_CONFIG = new TrajectoryConfig(2, 5)
            .setKinematics(Chassis.getInstance().swerveDriveKinematics);

    public Autonomous initialize(){
        defaultPath = NoOperation.NO_OP.get(SWERVE_CONFIG);
        addAuto(defaultPath);

        Arrays.stream(RedSideAuto.values()).forEach((ac) -> addAuto(ac.get(SWERVE_CONFIG)));
        Arrays.stream(BlueSideAuto.values()).forEach((ac) -> addAuto(ac.get(SWERVE_CONFIG)));

        selectionAutoChange(autoPathChooser.getSelected());
        SmartDashboard.putData("AutonomousChooser", autoPathChooser);

        NetworkTableInstance netInstance = NetworkTableInstance.getDefault();
        StringSubscriber subscriber = netInstance.getTable("SmartDashboard").getSubTable("AutonomousChooser").getStringTopic("selected").subscribe("defaultAutoCommand");
        netInstance.addListener(subscriber, EnumSet.of(NetworkTableEvent.Kind.kValueAll), e -> {
            selectionAutoChange(autoCommands.get((String) e.valueData.value.getValue()));
        });

        return this;
    }

    public void addAuto(Path path) {
        autoCommands.put(path.getName(), path);

        if (path == defaultPath) {
            autoPathChooser.setDefaultOption(path.getName(), path);
            //selectionAutoChange(auto);
        } else {
            autoPathChooser.addOption(path.getName(), path);
        }
    }

    /**
     * Listener for "AutonomousChooser/selected" change.
     */
    private void selectionAutoChange(Path path) {
        selectedPathCommand = path.build(Chassis.getInstance().getPose());
//        logger.info(selectedPathCommand.toString());
        selectedPathCommand.onSelection();
    }

    public Command getSelectedCommand() {
        return selectedPathCommand;
    }
}
