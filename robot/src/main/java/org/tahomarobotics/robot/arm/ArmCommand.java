package org.tahomarobotics.robot.arm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.util.ChartData;

import java.util.ArrayList;
import java.util.List;

public class ArmCommand extends CommandBase {

    private static final Logger logger = LoggerFactory.getLogger(ArmCommand.class);
    private boolean canceled = false;

    private List<ArmState> armStates;

    private final ArmFeedForward feedForward = new ArmFeedForward();
    ArmKinematics kinematics = new ArmKinematics();
    Timer timer = new Timer();
    ArmState armState;

    //ChartData chartData = new ChartData("Position", "Time", "Inches", new String[]{"Expected X", "Actual X", "Expected Y", "Actual Y"});
    ChartData chartData = new ChartData("Voltage", "Time", "Voltage", new String[]{"Shoulder Volts", "Elbow Volts"});
    //ChartData speedChart = new ChartData("Velocity", "Time", "Meters (m/s)", new String[]{"Shoulder Velocity","Elbow Velocity"});
    ChartData angleChart = new ChartData("Angles", "Time", "Degrees", new String[]{"Expected Shoulder Angle (Degrees)",
                                                                                                  "Expected Elbow Angle (Degrees)",
                                                                                                  "Actual Shoulder Angle (Degrees)",
                                                                                                  "Actual Elbow Angle (Degrees)"});
    private final ArmSubsystemIF arm = Arm.getInstance();

    private static final TrajectoryConfig TRAJ_CFG = new TrajectoryConfig(0.1, 0.1);
    private static final List<Translation2d> NONE = List.of();

    private static final Rotation2d UP = new Rotation2d(90);
    private static final Rotation2d DOWN = new Rotation2d(-90);
    private static final Rotation2d FWD = new Rotation2d(0);
    private static final Rotation2d REV = new Rotation2d(180);

    private static final Translation2d START = new Translation2d(Units.inchesToMeters(20.9), Units.inchesToMeters(-13.3));

    private static ArmState createStartState() {
        try {
             return new ArmKinematics().inverseKinematics(0, START);
        } catch (ArmKinematics.KinematicsException e ) {
            return new ArmState();
        }
    }
    static final ArmState START_STATE = createStartState();

    private static final Translation2d STOW = new Translation2d(Units.inchesToMeters(12.3), Units.inchesToMeters(0.8));
    private static final Translation2d HIGH = new Translation2d(Units.inchesToMeters(48), Units.inchesToMeters(24));

    public static final ArmCommand START_TO_STOW_ARM_COMMAND = new ArmCommand(new Pose2d(START, UP), NONE, new Pose2d(STOW, UP), TRAJ_CFG);

    public static final ArmCommand STOW_TO_HIGH_ARM_COMMAND = new ArmCommand(new Pose2d(STOW, FWD), NONE, new Pose2d(HIGH, FWD), TRAJ_CFG);

    public static final ArmCommand HIGH_TO_STOW_ARM_COMMAND = new ArmCommand(new Pose2d(HIGH, REV), NONE, new Pose2d(STOW, REV), TRAJ_CFG);


    private final double COMMAND_TIMEOUT;

    public ArmCommand(Trajectory trajectory) {

        COMMAND_TIMEOUT = trajectory.getTotalTimeSeconds() + 2.0; // 2 additional seconds

        //if (!kinematics.validateTrajectory(trajectory)) {
        //    cancelCommand();
        //}

        armStates = new ArrayList<>();
        int size = trajectory.getStates().size();
        for (int i = 0; i < size; i++) {
         Trajectory.State s = trajectory.getStates().get(i);
            try {
                ArmState armState = kinematics.inverseKinematics(s.timeSeconds, s.poseMeters);
                if (i >= size-1 ) {
                    armState = new ArmState(armState.time,
                            new ArmState.JointState(armState.shoulder.position(), 0 , 0),
                            new ArmState.JointState(armState.elbow.position(), 0 , 0));
                }
                System.out.println(armState);
                armStates.add(armState);
            } catch (ArmKinematics.KinematicsException e) {
                logger.error("Arm trajectory failed to work with kinematics", e);
                cancelCommand();
            }
        }
System.out.println();
        for (double t = 0; t < COMMAND_TIMEOUT; t += 0.020) {
            ArmState a = ArmState.sample(t, armStates);
            System.out.println(a);
        }
        System.out.println();System.out.println();
        addRequirements(arm);
    }
    public ArmCommand(Pose2d start, List<Translation2d> mid, Pose2d end, TrajectoryConfig config) {
        this(TrajectoryGenerator.generateTrajectory(start, mid, end, config));
    }

    public void cancelCommand() {
        canceled = true;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        chartData.clear();
        angleChart.clear();
        armState = armStates.get(0);
    }

    @Override
    public void execute() {
        double time = timer.get();
        ArmState desiredState = ArmState.sample(time, armStates);
        arm.setArmState(desiredState);

//        armState = desiredState;
        double[] voltages = arm.getVoltages();
        double[] data = {timer.get(), voltages[0], voltages[1]};
        chartData.addData(data);
//        double[] speedData = {timer.get(), armState.shoulder.velocity(), armState.shoulder.acceleration(), armState.elbow.velocity(), armState.elbow.acceleration()};
//        speedChart.addData(speedData);

        // expected arm position vs real position
        double[] angleData = {timer.get(), Units.radiansToDegrees(desiredState.shoulder.position()),
                                           Units.radiansToDegrees(desiredState.elbow.position()),
                                           Units.radiansToDegrees(arm.getCurrentArmState().shoulder.position()),
                                           Units.radiansToDegrees(arm.getCurrentArmState().elbow.position())};
        angleChart.addData(angleData);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        SmartDashboard.putRaw("Arm Chart", chartData.serialize());
        //SmartDashboard.putRaw("Velocity Chart", speedChart.serialize());
        SmartDashboard.putRaw("Angle Chart", angleChart.serialize());
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= COMMAND_TIMEOUT || canceled;
    }
}
