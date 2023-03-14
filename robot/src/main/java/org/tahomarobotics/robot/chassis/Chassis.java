/**
 * Copyright 2023 Tahoma Robotics - http://tahomarobotics.org - Bear Metal 2046 FRC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including without
 * limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
 * Software, and to permit persons to whom the Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions
 * of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
 * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 */
package org.tahomarobotics.robot.chassis;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.SubsystemIF;
import org.tahomarobotics.robot.chassis.mk4i.MK4iChassisConstants;
import org.tahomarobotics.robot.chassis.rev.RevChassisConstants;
import org.tahomarobotics.robot.ident.RobotIdentity;
import org.tahomarobotics.robot.util.CalibrationData;
import org.tahomarobotics.robot.vision.Vision;

import java.util.ArrayList;
import java.util.List;

/**
 * Chassis Subsystem Class
 * Handles Drivetrain, Vision/Odometry, Balancing
 * @implNote If class exceeds 500 lines consider making subclasses...
 */

public class Chassis extends SubsystemBase implements SubsystemIF {

    private static final Logger logger = LoggerFactory.getLogger(Chassis.class);
    private static Chassis INSTANCE = new Chassis();

    public static Chassis getInstance() {
        return INSTANCE;
    }

    public boolean isFieldOriented = true;

    private final Pigeon2 pigeon2 = new Pigeon2(RobotMap.PIGEON);

    private final List<SwerveModuleIF> swerveModules;
    private final ChassisConstantsIF swerveConstants;

    private final SwerveDrivePoseEstimator poseEstimator;

    private final Field2d fieldPose = new Field2d();
    private final List<Pose2d> actualPath = new ArrayList<>();

    private final SwerveDriveKinematics swerveDriveKinematics;

    private final CalibrationData<Double[]> swerveCalibration;

    private final Vision vision;

    private Chassis() {
        // Configures the Chassis according to the current RobotID.
        swerveConstants = switch (RobotIdentity.getInstance().getRobotID()) {
            // configure Mk4I Swerve
            case PROTOTYPE -> new MK4iChassisConstants();
            // configure Rev Swerve
            case ALPHA, PRACTICE, COMPETITION ->  new RevChassisConstants();
        };

        // read calibration data
        swerveCalibration = new CalibrationData<>("SwerveCalibration", new Double[]{0d, 0d, 0d, 0d});

        // create swerve modules
        swerveModules = swerveConstants.createSwerveModules(swerveCalibration.get());

        swerveDriveKinematics = new SwerveDriveKinematics(
                swerveModules.stream()
                        .map(SwerveModuleIF::getPositionOffset)
                        .toArray(Translation2d[]::new)
        );

        /*
        Pose Estimator is configured further down the line, and utilizes the previously set swerveConstants.
         */
        poseEstimator = new SwerveDrivePoseEstimator(
                swerveDriveKinematics,
                getGyroRotation(),
                getSwerveModulePositions(),
                new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.02),
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.01)
        );

        vision = new Vision((pose, time) -> {
//            var poseDiff = getPose().minus(pose);
//            double diff = Math.sqrt(Math.pow(poseDiff.getX(), 2) + Math.pow(poseDiff.getY(), 2));
//            if (diff > VisionConstants.RESET_THRESHOLD) {
//                poseEstimator.resetPosition(getGyroRotation(), getSwerveModulePositions(), pose);
//            } else {
//                poseEstimator.addVisionMeasurement(pose, time);
//            }
        });
    }


    /*
    Alignment Method
    Aligns all modules according to offsets.
     */
    public void finalizeCalibration() {
        swerveCalibration.set(
                swerveModules.stream()
                .map(SwerveModuleIF::finalizeCalibration)
                .toArray(Double[]::new)
        );
    }

    /*
    Zeroes all offsets, used for offset configuration.
     */
    public void initializeCalibration() { swerveModules.forEach(SwerveModuleIF::initializeCalibration); }

    /*
    Updates each offset to the new one defined in DriverStation.
     */
    public void cancelCalibration() { swerveModules.forEach(SwerveModuleIF::cancelCalibration); }

    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(pigeon2.getYaw());
    }
    public Rotation2d getPitch() {
        return Rotation2d.fromDegrees(pigeon2.getPitch());
    }
    public Rotation2d getRoll() {
        return Rotation2d.fromDegrees(pigeon2.getRoll());
    }
    private void zeroGyro(){pigeon2.setYaw(0.0);}

    private Rotation2d getGyroRotation(){
        return Rotation2d.fromDegrees(pigeon2.getYaw());
    }

    public void toggleOrientation(){
        isFieldOriented = !isFieldOriented;
    }

    public Chassis initialize() {
        SmartDashboard.putData("Align Swerves", new AlignSwerveCommand());

        zeroGyro();

        poseEstimator.resetPosition(getGyroRotation(), getSwerveModulePositions(), new Pose2d(0.0,0.0, new Rotation2d(0.0)));

        SmartDashboard.putData(fieldPose);

        return this;
    }

    @Override
    public void periodic() {
        poseEstimator.update(getGyroRotation(), getSwerveModulePositions());
        fieldPose.setRobotPose(getPose());
        swerveModules.forEach(SwerveModuleIF::displayPosition);

        SmartDashboard.putString("Gyro Yaw", getYaw().toString());
        SmartDashboard.putString("Gyro Pitch", getPitch().toString());
        SmartDashboard.putString("Gyro Roll", getRoll().toString());
        SmartDashboard.putString("Pose", getPose().toString());
    }

    public Pose2d getPose(){
        return poseEstimator.getEstimatedPosition();
    }

    private SwerveModulePosition[] getSwerveModulePositions() {
        return swerveModules.stream().map(SwerveModuleIF::getPosition).toArray(SwerveModulePosition[]::new);
    }

    public double getAvgVelocity() {
        return swerveModules.stream().mapToDouble(SwerveModuleIF::getDriveVelocity).sum()/4d;
    }

    public void setSwerveStates(SwerveModuleState[] states) {
        for(int i = 0; i < states.length; i++) swerveModules.get(i).setDesiredState(states[i]);
    }

    public void drive(ChassisSpeeds velocity) { drive(velocity, isFieldOriented); }

    public void drive(ChassisSpeeds velocity, boolean fieldRelative) {

        if (fieldRelative) {
            velocity = ChassisSpeeds.fromFieldRelativeSpeeds(velocity, getPose().getRotation());
        }

        var swerveModuleStates = swerveDriveKinematics.toSwerveModuleStates(velocity);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, swerveConstants.maxAttainableMps());

        setSwerveStates(swerveModuleStates);
    }

    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(getGyroRotation(), getSwerveModulePositions(), pose);
    }

    public void orientToZeroHeading() {
        Pose2d pose = getPose();
        resetOdometry(new Pose2d(pose.getTranslation(), new Rotation2d(0)));
    }

    public ChassisConstantsIF getSwerveConstants() { return swerveConstants; }

    public SwerveDriveKinematics getSwerveDriveKinematics() {
        return swerveDriveKinematics;
    }

    @Override
    public void simulationPeriodic() {
        final double dT = 0.02;

        ChassisSpeeds speeds = swerveDriveKinematics.toChassisSpeeds(swerveModules.stream()
                .map(SwerveModuleIF::getState)
                .toArray(SwerveModuleState[]::new));


        pigeon2.getSimCollection().addHeading(dT * Units.radiansToDegrees(speeds.omegaRadiansPerSecond));
    }

    public void displayAbsolutePositions() { swerveModules.forEach(SwerveModuleIF::displayPosition); }

    public void updateTrajectory(List<Trajectory> trajectories) {

        // clear out any previous paths
        for (int i = 0; i < 10; i++) {
            fieldPose.getObject("traj" + i).setTrajectory(new Trajectory());
        }
        fieldPose.getObject("robot-path").setTrajectory(new Trajectory());
        actualPath.clear();

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

    public Field2d getFieldView() {
        return fieldPose;
    }

    @Override
    public void onDisabledInit() {
        swerveModules.forEach(s -> s.setDriveVoltage(0));
    }
}