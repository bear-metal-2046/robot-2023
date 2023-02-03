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
import org.tahomarobotics.robot.chassis.config.MK4iConstants;
import org.tahomarobotics.robot.chassis.config.REVMaxConstants;
import org.tahomarobotics.robot.chassis.config.SwerveConstantsIF;
import org.tahomarobotics.robot.chassis.module.MAXSwerveModule;
import org.tahomarobotics.robot.chassis.module.MK4iSwerveModule;
import org.tahomarobotics.robot.chassis.module.SwerveModuleIF;
import org.tahomarobotics.robot.ident.RobotIdentity;

import java.util.ArrayList;
import java.util.List;

/**
 * Chassis Subsystem Class
 * Handles Drivetrain, Vision/Odometry, Balancing
 * @implNote If class exceeds 500 lines consider making subclasses...
 */

public class Chassis extends SubsystemBase {

    private static final Logger logger = LoggerFactory.getLogger(Chassis.class);

    private static final Chassis INSTANCE = new Chassis().initialize();

    public static Chassis getInstance() { return INSTANCE; }

    public boolean isFieldOriented = true;

    private final Pigeon2 pigeon2 = new Pigeon2(RobotMap.PIGEON);

    private final SwerveModuleIF frontLeftSwerveModule;
    private final SwerveModuleIF frontRightSwerveModule;
    private final SwerveModuleIF backLeftSwerveModule;
    private final SwerveModuleIF backRightSwerveModule;

    private final SwerveConstantsIF swerveConstants;

    private final SwerveDrivePoseEstimator poseEstimator;

    private final Field2d fieldPose = new Field2d();
    private final List<Pose2d> actualPath = new ArrayList<>();

    /*
    Base Constructor
    Contains some example comments in order to help with understanding.
     */
    private Chassis() {
        /*
        Configures the Chassis according to the current RobotID.
         */
        switch (RobotIdentity.getInstance().getRobotID()) {
            case PROTOTYPE:

                // configure Mk4I Swerve
                /*
                The current swerveConstants must be defined in order for Chassis to configure properly.
                These constants classes should not vary per robot, and are determined by the swerve module.
                It is possible that in the future there may also be a "RobotConstant" class that IS robot
                specific. However, at this time that is not needed.
                 */
                swerveConstants = new MK4iConstants();
                /*
                Each Swerve Module must be configured according to its module.
                IMPORTANT NOTE: ALL FOUR MODULES MUST BE THE SAME. (Currently either MK4i or REVMax)
                 */
                frontRightSwerveModule = new MK4iSwerveModule(MK4iConstants.FRONT_RIGHT_SWERVE_CONFIG);
                frontLeftSwerveModule = new MK4iSwerveModule(MK4iConstants.FRONT_LEFT_SWERVE_CONFIG);
                backRightSwerveModule = new MK4iSwerveModule(MK4iConstants.BACK_RIGHT_SWERVE_CONFIG);
                backLeftSwerveModule = new MK4iSwerveModule(MK4iConstants.BACK_LEFT_SWERVE_CONFIG);

                break;

            case ALPHA:
            case COMPETITION :
            default : // always default to COMPETITION robot as this is the most important configuration

                // configure REV Swerve
                swerveConstants = new REVMaxConstants();
                frontRightSwerveModule = new MAXSwerveModule(REVMaxConstants.FRONT_RIGHT_SWERVE_CONFIG);
                frontLeftSwerveModule = new MAXSwerveModule(REVMaxConstants.FRONT_LEFT_SWERVE_CONFIG);
                backLeftSwerveModule = new MAXSwerveModule(REVMaxConstants.BACK_LEFT_SWERVE_CONFIG);
                backRightSwerveModule = new MAXSwerveModule(REVMaxConstants.BACK_RIGHT_SWERVE_CONFIG);

                break;
        }

        /*
        Pose Estimator is configured further down the line, and utilizes the previously set swerveConstants.
         */
        poseEstimator = new SwerveDrivePoseEstimator(
                swerveConstants.getSwerveDriveKinematics(),
                getGyroRotation(),
                getSwerveModulePositions(),
                new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.02),
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.01)
        );
    }


    /*
    Alignment Method
    Aligns all modules according to offsets.
     */
    public void align() {
        frontLeftSwerveModule.align();
        frontRightSwerveModule.align();
        backLeftSwerveModule.align();
        backRightSwerveModule.align();
    }

    /*
    Zeroes all offsets, used for offset configuration.
     */
    public void zeroOffsets() {
        frontLeftSwerveModule.zeroOffset();
        frontRightSwerveModule.zeroOffset();
        backLeftSwerveModule.zeroOffset();
        backRightSwerveModule.zeroOffset();
    }

    /*
    Updates each offset to the new one defined in DriverStation.
     */
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
//
        SmartDashboard.putNumber("FL " +
                "offset", frontLeftSwerveModule.getAbsoluteAngle());
        SmartDashboard.putNumber("FR offset", frontRightSwerveModule.getAbsoluteAngle());
        SmartDashboard.putNumber("BL offset", backLeftSwerveModule.getAbsoluteAngle());
        SmartDashboard.putNumber("BR offset", backRightSwerveModule.getAbsoluteAngle());
//        SmartDashboard.putNumber("Steer ABS angle not method", frontRightSwerveModule.getSteerABSEncoder().getPosition());
    }

    public Pose2d getPose(){
        return poseEstimator.getEstimatedPosition();
    }

    //TODO Abstract Impl Done
    private SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[] {
                frontLeftSwerveModule.getPosition(),
                frontRightSwerveModule.getPosition(),
                backLeftSwerveModule.getPosition(),
                backRightSwerveModule.getPosition()
        };
    }

    //TODO Abstract Impl Done
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

        var swerveModuleStates = swerveConstants.getSwerveDriveKinematics().toSwerveModuleStates(velocity);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, swerveConstants.maxAttainableMps());

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

    public SwerveConstantsIF getSwerveConstants() { return swerveConstants; }

    @Override
    public void simulationPeriodic() {
        final double dT = 0.02;

        ChassisSpeeds speeds = swerveConstants.toChassisSpeeds(
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