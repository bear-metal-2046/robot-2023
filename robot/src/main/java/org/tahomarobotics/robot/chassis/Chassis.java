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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.RobotMap;

import java.util.ArrayList;
import java.util.List;

/**
 * Chassis Subsystem Class
 * Handles Drivetrain, Vision/Odometry, Balancing
 * @implNote If class exceeds 500 lines consider making subclasses...
 */

public class Chassis extends SubsystemBase {
    private static final Logger logger = LoggerFactory.getLogger(Chassis.class);

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

    private Chassis() {

    }

    public void align() {
        frontLeftSwerveModule.align();
        frontRightSwerveModule.align();
        backLeftSwerveModule.align();
        backRightSwerveModule.align();
    }

    public void zeroOffsets() {
        frontLeftSwerveModule.zeroOffset();
        frontRightSwerveModule.zeroOffset();
        backLeftSwerveModule.zeroOffset();
        backRightSwerveModule.zeroOffset();
    }

    public void updateOffsets() {
        frontLeftSwerveModule.updateOffset();
        frontRightSwerveModule.updateOffset();
        backLeftSwerveModule.updateOffset();
        backRightSwerveModule.updateOffset();
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
        updateOffsets();
        return this;
    }

    @Override
    public void periodic() {
        poseEstimator.update(getGyroRotation(), getSwerveModulePositions());
        fieldPose.setRobotPose(getPose());
    }

    public Pose2d getPose(){
        return poseEstimator.getEstimatedPosition();
    }

    private SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[] {
                frontLeftSwerveModule.getPosition(),
                frontRightSwerveModule.getPosition(),
                backLeftSwerveModule.getPosition(),
                backRightSwerveModule.getPosition()
        };
    }

    private void setSwerveStates(SwerveModuleState[] states){
        frontLeftSwerveModule.setDesiredState(states[0]);
        frontRightSwerveModule.setDesiredState(states[1]);
        backLeftSwerveModule.setDesiredState(states[2]);
        backRightSwerveModule.setDesiredState(states[3]);
    }

    public void drive(ChassisSpeeds velocity) {
        drive(velocity, isFieldOriented);

    }

    private void drive(ChassisSpeeds velocity, boolean fieldRelative) {

        if (fieldRelative) {
            velocity = ChassisSpeeds.fromFieldRelativeSpeeds(velocity, getPose().getRotation());
        }

        var swerveModuleStates = ChassisConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(velocity);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, ChassisConstants.MAX_VELOCITY_MPS);

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

    public void displayAbsolutePositions() {
        frontLeftSwerveModule.displayPosition();
        frontRightSwerveModule.displayPosition();
        backLeftSwerveModule.displayPosition();
        backRightSwerveModule.displayPosition();
    }
}