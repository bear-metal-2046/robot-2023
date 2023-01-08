package org.tahomarobotics.robot.chassis;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.Vision.ATs.AprilTagVision;

import java.util.ArrayList;
import java.util.List;

public class Chassis extends SubsystemBase {
    private static final Chassis INSTANCE = new Chassis();
    public static Chassis getInstance() { return INSTANCE; }

    public boolean isFieldOriented = true;

    private final Pigeon2 pigeon2 = new Pigeon2(RobotMap.PIGEON);

    private final SwerveModule lFSwerveModule = new SwerveModule(ChassisConstants.L_F_SWERVE_CONFIG);
    private final SwerveModule rFSwerveModule = new SwerveModule(ChassisConstants.R_F_SWERVE_CONFIG);
    private final SwerveModule lBSwerveModule = new SwerveModule(ChassisConstants.L_B_SWERVE_CONFIG);
    private final SwerveModule rBSwerveModule = new SwerveModule(ChassisConstants.R_B_SWERVE_CONFIG);

    private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
            getGyroRotation(),
            new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
            ChassisConstants.SWERVE_DRIVE_KINEMATICS,
            // Three below vectors are standard deviations
            // Increasing the values below decreases trust
            // Below values are from the WPILIB article (Advanced Controls -> State-Space -> Pose estimators)

            // Model std devs (x, y, theta). Units are meters/radians
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.02),
            // Gyro Measurement (theta). Units are radians
            new MatBuilder<>(Nat.N1(), Nat.N1()).fill(0.01),
            // Vision measurements std devs (x, y, theta). Units are meters/radians
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.01)
    );

    private final Field2d fieldPose = new Field2d();
    private final List<Pose2d> actualPath = new ArrayList<>();

    private double lastVisionUpdate = WPIUtilJNI.now() * 1.0e-6;

    private final AprilTagVision vision = new AprilTagVision(poseEstimator::addVisionMeasurement);

    private Chassis() {

    }

    public void toggleOriented() {
        isFieldOriented = !isFieldOriented;
    }

    private Rotation2d getYaw() {
        return Rotation2d.fromDegrees(pigeon2.getYaw());
    }

    private Rotation2d getPitch() {
        return Rotation2d.fromDegrees(pigeon2.getPitch());
    }

    /**
     * Checks to see if robot is level (within reason)
     * (2 degrees of leniency)
     * @return if the robot is level.
     */
    public boolean isLevel() {
        return getPitch().getDegrees() <= 2;
    }

    private void zeroGyro(){pigeon2.setYaw(0.0);}

    public void toggleOrientation(){
        isFieldOriented = !isFieldOriented;
    }

    private Rotation2d getGyroRotation(){
        return Rotation2d.fromDegrees(pigeon2.getYaw());
    }

    public Chassis initialize(){
        zeroGyro();
        poseEstimator.resetPosition(getGyroRotation(),
                new SwerveModulePosition[]{lFSwerveModule.getPosition(), rFSwerveModule.getPosition(), lBSwerveModule.getPosition(), rBSwerveModule.getPosition()}
                ,new Pose2d(0.0,0.0, new Rotation2d(0.0)));
        SmartDashboard.putData(fieldPose);

        return this;
    }

    @Override
    public void periodic() {
        var pose = poseEstimator.update(getGyroRotation(),
                new SwerveModulePosition[]{lFSwerveModule.getPosition(), rFSwerveModule.getPosition(), lBSwerveModule.getPosition(), rBSwerveModule.getPosition()});

        SmartDashboard.putString("pose", pose.toString());

        SmartDashboard.putNumber("L-F Steer Angle", Units.radiansToDegrees(lFSwerveModule.getSteerAngle()));
        SmartDashboard.putNumber("R-F Steer Angle", Units.radiansToDegrees(rFSwerveModule.getSteerAngle()));
        SmartDashboard.putNumber("L-B Steer Angle",  Units.radiansToDegrees(lBSwerveModule.getSteerAngle()));
        SmartDashboard.putNumber("R-B Steer Angle", Units.radiansToDegrees(rBSwerveModule.getSteerAngle()));

        fieldPose.setRobotPose(getPose());
    }

    public Pose2d getPose(){
        return poseEstimator.getEstimatedPosition();
    }

    public void setSwerveStates(SwerveModuleState[] states){
        lFSwerveModule.setDesiredState(states[0]);
        rFSwerveModule.setDesiredState(states[1]);
        lBSwerveModule.setDesiredState(states[2]);
        rBSwerveModule.setDesiredState(states[3]);
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
        lFSwerveModule.setDriveVoltage(voltage);
        rFSwerveModule.setDriveVoltage(voltage);
        lBSwerveModule.setDriveVoltage(voltage);
        rBSwerveModule.setDriveVoltage(voltage);
        return lFSwerveModule.getVelocity();
    }

    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(getGyroRotation(), new SwerveModulePosition[]{lFSwerveModule.getPosition(), rFSwerveModule.getPosition(), lBSwerveModule.getPosition(), rBSwerveModule.getPosition()}, pose);
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
                lFSwerveModule.getState(), rFSwerveModule.getState(),
                lBSwerveModule.getState(), rBSwerveModule.getState());

        pigeon2.getSimCollection().addHeading(dT * Units.radiansToDegrees(speeds.omegaRadiansPerSecond));
    }

}