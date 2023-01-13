package org.tahomarobotics.robot.chassis;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.tahomarobotics.robot.OI.OI;
import org.tahomarobotics.robot.RobotMap;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;

/**
 * Chassis Subsystem Class
 * Handles Drivetrain, Vision/Odometry, Balancing
 * @implNote If class exceeds 500 lines consider making subclasses...
 */

public class Chassis extends SubsystemBase {
    private static final Chassis INSTANCE = new Chassis();
    public static Chassis getInstance() { return INSTANCE; }

    public boolean isFieldOriented = true;

    private final Pigeon2 pigeon2 = new Pigeon2(RobotMap.PIGEON);

    private final SwerveModule frontLeftSwerveModule = new SwerveModule(ChassisConstants.FRONT_LEFT_SWERVE_CONFIG);
    private final SwerveModule frontRightSwerveModule = new SwerveModule(ChassisConstants.FRONT_RIGHT_SWERVE_CONFIG);
    private final SwerveModule backLeftSwerveModule = new SwerveModule(ChassisConstants.BACK_LEFT_SWERVE_CONFIG);
    private final SwerveModule backRightSwerveModule = new SwerveModule(ChassisConstants.BACK_RIGHT_SWERVE_CONFIG);

    private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
            ChassisConstants.SWERVE_DRIVE_KINEMATICS,
            getGyroRotation(),
            getSwerveModulePositions(),
            new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.02),
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.01)
    );

    private final Field2d fieldPose = new Field2d();
    private final List<Pose2d> actualPath = new ArrayList<>();

    // UNCOMMENT
//    private final AprilTagVision vision = new AprilTagVision(poseEstimator::addVisionMeasurement);

    public void closeVision() {
//        vision.close();
    }

    private Chassis() {

    }

    private Rotation2d getYaw() {
        return Rotation2d.fromDegrees(pigeon2.getYaw());
    }

    private Rotation2d getPitch() {
        return Rotation2d.fromDegrees(pigeon2.getPitch());
    }

    /**
     * Painful level checking, works now though.
     * TODO there is definitely a better way to do this
     * @return if the robot is level. With a bit of leniency.
     */
    public final boolean isLevel() {
        return (getPitch().getDegrees() < 1 && getPitch().getDegrees() > -1) && (getYaw().getDegrees() < 1 && getYaw().getDegrees() > -1);
    }

    private void zeroGyro(){pigeon2.setYaw(0.0);}

    private Rotation2d getGyroRotation(){
        return Rotation2d.fromDegrees(pigeon2.getYaw());
    }

    public void toggleOrientation(){
        isFieldOriented = !isFieldOriented;
    }

    public Chassis initialize(){
        zeroGyro();
        poseEstimator.resetPosition(getGyroRotation(),
                getSwerveModulePositions()
                ,new Pose2d(0.0,0.0, new Rotation2d(0.0)));
        SmartDashboard.putData(fieldPose);

//        SmartDashboard.putData("MOD SELECTOR", selector);
//
//        selector.addOption("1", 1);
//        selector.addOption("2", 2);
//        selector.addOption("3", 3);
//        selector.addOption("4", 4);
//
//        int handler = NetworkTableInstance.getDefault().getTable("SmartDashboard").addListener(
//                "MOD SELECTOR",
//                EnumSet.of(NetworkTableEvent.Kind.kValueAll),
//                (table, key, event) -> {
//                    Integer selected = selector.getSelected();
//                    DriverStation.reportError("Picked " + selected, false);
//                    selectMod(selected);
//                }
//        );

        return this;
    }

    public void selectMod(int i) {
        selectedModule = i;
    }

    public SendableChooser<Integer> selector = new SendableChooser<>();
    public int selectedModule = 1;

    @Override
    public void periodic() {
        var pose = poseEstimator.update(getGyroRotation(),
                getSwerveModulePositions());

//        SmartDashboard.putString("pose", pose.toString());
//
        SmartDashboard.putNumber("L-F Steer Angle", Units.radiansToDegrees(frontLeftSwerveModule.getSteerAngle()));
        SmartDashboard.putNumber("R-F Steer Angle", Units.radiansToDegrees(frontRightSwerveModule.getSteerAngle()));
        SmartDashboard.putNumber("L-B Steer Angle",  Units.radiansToDegrees(backLeftSwerveModule.getSteerAngle()));
        SmartDashboard.putNumber("R-B Steer Angle", Units.radiansToDegrees(backRightSwerveModule.getSteerAngle()));

        fieldPose.setRobotPose(getPose());

        SmartDashboard.putNumber("LF ABSOLUTE ANGLE :)", frontLeftSwerveModule.getAbsoluteAngle());
        SmartDashboard.putNumber("RF ABSOLUTE ANGLE :)", frontRightSwerveModule.getAbsoluteAngle());
        SmartDashboard.putNumber("LB ABSOLUTE ANGLE :)", backLeftSwerveModule.getAbsoluteAngle());
        SmartDashboard.putNumber("RB ABSOLUTE ANGLE :)", backRightSwerveModule.getAbsoluteAngle());

        SmartDashboard.putNumber("steer angle", frontRightSwerveModule.getSteerAngle());
        SmartDashboard.putNumber("ABS angle", frontRightSwerveModule.getAbsoluteAngle());


//        double jx = OI.getInstance().getDriveRightXJoystick();
//        double jy = OI.getInstance().getDriveLeftYJoystick();
//        selectedModule = selector.getSelected() != null ? selector.getSelected() : 0;
//        SmartDashboard.putNumber("JEX", jx);
//        SmartDashboard.putNumber("JWHY", jy);
//        SmartDashboard.putNumber("SelMod", selectedModule);
//        switch (selectedModule) {
//            case 1 -> {
//                frontLeftSwerveModule.setDesiredState(new SwerveModuleState(jy, Rotation2d.fromRadians(jx * 2 * Math.PI)));
//            }
//            case 2 -> {
//                frontRightSwerveModule.setDesiredState(new SwerveModuleState(jy, Rotation2d.fromRadians(jx * 2 * Math.PI)));
//            }
//            case 3 -> {
//                backLeftSwerveModule.setDesiredState(new SwerveModuleState(jy, Rotation2d.fromRadians(jx * 2 * Math.PI)));
//            }
//            case 4 -> {
//                backRightSwerveModule.setDesiredState(new SwerveModuleState(jy, Rotation2d.fromRadians(jx * 2 * Math.PI)));
//            }
//        }
    }

    public Pose2d getPose(){
        return poseEstimator.getEstimatedPosition();
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[]{frontLeftSwerveModule.getPosition(), frontRightSwerveModule.getPosition(), backLeftSwerveModule.getPosition(), backRightSwerveModule.getPosition()};
    }

    public void setSwerveStates(SwerveModuleState[] states){
        frontLeftSwerveModule.setDesiredState(states[0]);
        frontRightSwerveModule.setDesiredState(states[1]);
        backLeftSwerveModule.setDesiredState(states[2]);
        backRightSwerveModule.setDesiredState(states[3]);
    }

    public void drive(double xSpeed, double ySpeed, double rot) {
        drive(xSpeed, ySpeed, rot, isFieldOriented);
    }

    private void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        var swerveModuleStates =
                ChassisConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(fieldRelative ?
                        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getPose().getRotation()) :
                        new ChassisSpeeds(xSpeed, ySpeed, rot));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, ChassisConstants.MAX_VELOCITY_MPS * ChassisConstants.VELOCITY_MULTIPLIER);

        setSwerveStates(swerveModuleStates);
    }

    public void stop() {
        setDriveVoltage(0);
    }

    public double setDriveVoltage(double voltage) {
        frontLeftSwerveModule.setDriveVoltage(voltage);
        frontRightSwerveModule.setDriveVoltage(voltage);
        backLeftSwerveModule.setDriveVoltage(voltage);
        backRightSwerveModule.setDriveVoltage(voltage);
        return frontLeftSwerveModule.getVelocity();
    }

    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(getGyroRotation(), getSwerveModulePositions(), pose);
    }

    /**
     * Sends a list of trajectories to the SmartDashboard
     *
     */
    public void updateTrajectory(List<Trajectory> trajectories) {

        String name = SendableRegistry.getName(fieldPose);
        for(int i = 0; i < 10; i++) {
            SmartDashboard.getEntry(name + "/traj" + i).unpublish();
        }
        actualPath.clear();
        SmartDashboard.getEntry(name + "/robot-path").unpublish();

        fieldPose.setRobotPose(trajectories == null ? getPose() : trajectories.get(0).getInitialPose());

        if (trajectories != null) {
            for (int i = 0; i < trajectories.size(); i++) {
                fieldPose.getObject("traj" + i).setTrajectory(trajectories.get(i));
            }
        }

    }

    public void updateActualTrajectory(List<Pose2d> actualTrajectory) {
        actualPath.addAll(actualTrajectory);
        fieldPose.getObject("robot-path").setPoses(actualPath);
    }

    public void orientToZeroHeading() {
        Pose2d pose = getPose();
        resetOdometry(new Pose2d(pose.getTranslation(), new Rotation2d(0)));
    }

    @Override
    public void simulationPeriodic() {
        final double dT = 0.02;

        ChassisSpeeds speeds = ChassisConstants.SWERVE_DRIVE_KINEMATICS.toChassisSpeeds(
                frontLeftSwerveModule.getState(), frontRightSwerveModule.getState(),
                backLeftSwerveModule.getState(), backRightSwerveModule.getState());

        pigeon2.getSimCollection().addHeading(dT * Units.radiansToDegrees(speeds.omegaRadiansPerSecond));
    }

}