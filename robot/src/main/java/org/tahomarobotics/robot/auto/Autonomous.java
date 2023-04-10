package org.tahomarobotics.robot.auto;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.SubsystemIF;
import org.tahomarobotics.robot.arm.ArmMovements;
import org.tahomarobotics.robot.auto.Cable.CableTwoPiece;
import org.tahomarobotics.robot.auto.Cable.CableTwoPieceEngage;
import org.tahomarobotics.robot.auto.Cable.CableWeirdThreePiece;
import org.tahomarobotics.robot.auto.Loading.LoadingThreePiece;
import org.tahomarobotics.robot.auto.Loading.LoadingTwoPiece;
import org.tahomarobotics.robot.auto.Loading.LoadingTwoPieceCollect;
import org.tahomarobotics.robot.auto.Loading.LoadingTwoPieceEngage;
import org.tahomarobotics.robot.auto.Mid.MidEngage;

import java.util.EnumSet;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Function;

public class Autonomous extends SubsystemBase implements SubsystemIF {
    static class AutonomousOption {
        String name;
        Function<DriverStation.Alliance, Command> supplier;

        public AutonomousOption(
                String name,
                Function<DriverStation.Alliance, Command> supplier
        ) {
            this.name = name;
            this.supplier = supplier;
        }
    }

    /** Initialize this classes logger. */
    private static final Logger logger = LoggerFactory.getLogger(Autonomous.class);

    private static final Autonomous INSTANCE = new Autonomous();
    private static final double AUTO_STOW_TIMEOUT = 5.0;

    public static Autonomous getInstance(){
        return INSTANCE;
    }

    private final Map<String, AutonomousOption> autoCommands = new HashMap<>();
    private final SendableChooser<AutonomousOption> autoCommandChooser = new SendableChooser<>();
    private Command autonomousCommand;
    private AutoShuffleboard shuffleboard;

    private final Timer automaticStowTimer = new Timer();

    public Autonomous initialize(){
        shuffleboard = new AutoShuffleboard(autoCommandChooser);

        addAuto("No Operation", NoOperation::new);

        addAuto("Mid Engage", MidEngage::new);

        addAuto("Loading Two Piece", LoadingTwoPiece::new);
        addAuto("Loading Two Piece Engage", LoadingTwoPieceEngage::new);
        addAuto("Loading Two Piece Collect", LoadingTwoPieceCollect::new);
        addAuto("Loading Three Piece", LoadingThreePiece::new);

        addAuto("Cable Two Piece", CableTwoPiece::new);
        addAuto("Cable Two Piece Engage", CableTwoPieceEngage::new);
        addAuto("Cable Three Piece", CableWeirdThreePiece::new);

        selectionAutoChange(autoCommandChooser.getSelected());

        NetworkTableInstance netInstance = NetworkTableInstance.getDefault();
        StringSubscriber subscriber = netInstance.getTable("Shuffleboard").getSubTable("Auto/Auto Chooser").getStringTopic("selected").subscribe("defaultAutoCommand");
        netInstance.addListener(subscriber, EnumSet.of(NetworkTableEvent.Kind.kValueAll), e -> {
            selectionAutoChange(autoCommands.get((String) e.valueData.value.getValue()));
        });

        BooleanSubscriber allianceChange = netInstance.getTable("FMSInfo").getBooleanTopic("IsRedAlliance").subscribe(true);
        netInstance.addListener(allianceChange, EnumSet.of(NetworkTableEvent.Kind.kValueAll), e -> {
            selectionAutoChange(autoCommandChooser.getSelected());
        });

        return this;
    }

    @Override
    public void periodic() {
        shuffleboard.update();
    }

    private void addAuto(String name, Function<DriverStation.Alliance, Command> commandSup) {
        AutonomousOption opt = new AutonomousOption(name, commandSup);
        autoCommands.put(name, opt);

        if (name.equals("No Operation")) {
            autoCommandChooser.setDefaultOption(name, opt);
        } else {
            autoCommandChooser.addOption(name, opt);
        }
    }

    /**
     * Listener for "AutonomousChooser/selected" change.
     */
    private void selectionAutoChange(AutonomousOption opt) {
        if (opt == null) return;
        Command command = opt.supplier.apply(DriverStation.getAlliance());
        if (command instanceof AutonomousCommandIF) {
            ((AutonomousCommandIF) command).onSelection();
        }
    }

    private Command getSelectedCommand() {
        return autoCommandChooser.getSelected().supplier.apply(DriverStation.getAlliance());
    }

    @Override
    public void onDisabledInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
            automaticStowTimer.restart();
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
            if ( ! automaticStowTimer.hasElapsed(AUTO_STOW_TIMEOUT) ) {
                ArmMovements.createPositionToStowCommand().schedule();
            }
            autonomousCommand = null;
        }

    }
}
