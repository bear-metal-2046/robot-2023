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
package org.tahomarobotics.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.arm.ArmMoveCommand;
import org.tahomarobotics.robot.arm.ArmMovements;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.chassis.TeleopDriveCommand;
import org.tahomarobotics.robot.climb.ClimbSequence;
import org.tahomarobotics.robot.climb.ResetCommand;
import org.tahomarobotics.robot.grabber.CollectCommand;
import org.tahomarobotics.robot.grabber.Grabber;

import java.util.function.IntSupplier;

import static edu.wpi.first.wpilibj.XboxController.Button.*;

public final class OI implements SubsystemIF {
    private static final Logger logger = LoggerFactory.getLogger(OI.class);

    private static final OI INSTANCE = new OI();
    public static OI getInstance() {return INSTANCE;}
    private final Grabber grabber = Grabber.getInstance();

    private static final double ROTATIONAL_SENSITIVITY = 3;
    private static final double FORWARD_SENSITIVITY = 3;

    private final XboxController driveController = new XboxController(0);
    private final XboxController manipController = new XboxController(1);

    private final ClimbSequence climbSequence = new ClimbSequence();
    private final ResetCommand resetCommand = new ResetCommand();
    private ArmMoveCommand scoreCommand;
    private ArmMoveCommand collectUpCommand;
    private ArmMoveCommand collectDownCommand;

    private static boolean ifCone = false;
    private boolean ifReversed = false;
    private boolean ifUpCollectReversed = false;
    private boolean ifDownCollectReversed = false;
    private String scoreCommandName;
    private String upName;
    private String downName;
    private IntSupplier manipPOV;
    private enum ConeOrCube {
        CONE,
        CUBE;
    }
    private enum ScoringLevel {
        HIGH,
        MID,
        LOW;
    }
    private ScoringLevel level = ScoringLevel.LOW;
    private static ConeOrCube mode = ConeOrCube.CUBE;

    private OI() {

        Chassis chassis = Chassis.getInstance();

        chassis.setDefaultCommand(
                new TeleopDriveCommand(
                        () -> -desensitizePowerBased(driveController.getLeftY(), FORWARD_SENSITIVITY),
                        () -> -desensitizePowerBased(driveController.getLeftX(), FORWARD_SENSITIVITY),
                        () -> -desensitizePowerBased(driveController.getRightX(), ROTATIONAL_SENSITIVITY)
                )
        );

        grabber.setDefaultCommand(
                new CollectCommand(
                        driveController::getLeftTriggerAxis,
                        manipController::getLeftTriggerAxis,
                        driveController::getPOV
                )
        );
        //Robot Orientation
        JoystickButton AButton = new JoystickButton(driveController, 1);
        AButton.onTrue(new InstantCommand(chassis::orientToZeroHeading));

        JoystickButton BButton = new JoystickButton(driveController, 2);
        BButton.onTrue(new InstantCommand(chassis::toggleOrientation));

        //Climb
        JoystickButton climb = new JoystickButton(driveController, kStart.value);
        climb.onTrue(new InstantCommand(this::initiateClimb));

        JoystickButton reset = new JoystickButton(driveController, kBack.value);
        reset.onTrue(new InstantCommand(this::resetClimb));

        //Collecting
        JoystickButton XButton = new JoystickButton(driveController, kX.value);
        XButton.onTrue(new InstantCommand(() -> ifCone = !ifCone));

        JoystickButton driveRB = new JoystickButton(driveController, kRightBumper.value);
        driveRB.onTrue(new InstantCommand(() -> scheduleCollectCommand(collectUpCommand))
                .andThen(new InstantCommand(() -> ifUpCollectReversed = !ifUpCollectReversed)));

        JoystickButton driveLB = new JoystickButton(driveController, kLeftBumper.value);
        driveLB.onTrue(new InstantCommand(() -> scheduleCollectCommand(collectDownCommand))
                .andThen(new InstantCommand(() -> ifDownCollectReversed = !ifDownCollectReversed)));

        //Scoring
        JoystickButton manipLB = new JoystickButton(manipController, kRightBumper.value);
        manipLB.onTrue(new InstantCommand(() -> scheduleArmCommand(scoreCommand))
                .andThen(new InstantCommand(() -> ifReversed = !ifReversed)));

        manipPOV = manipController::getPOV;
    }


    public void periodic() {
        switch (manipPOV.getAsInt()) {
            case 0 -> level = ScoringLevel.HIGH;
            case 90 -> level = ScoringLevel.MID;
            case 180 -> level = ScoringLevel.LOW;
        }
        switch (mode) {
            case CONE -> {
                if(!ifCone) mode = ConeOrCube.CUBE;
                switch (level) {
                    case HIGH -> {
                        scoreCommand = ifReversed ? ArmMovements.HIGH_POLE_TO_STOW : ArmMovements.STOW_TO_HIGH_POLE;
                        scoreCommandName = ifReversed ? "ArmMovements.HIGH_POLE_TO_STOW" : "ArmMovements.STOW_TO_HIGH_POLE";
                    }
                    case MID -> {
                        scoreCommand = ifReversed ? ArmMovements.MID_POLE_TO_STOW : ArmMovements.STOW_TO_MID_POLE;
                        scoreCommandName = ifReversed ? "ArmMovements.MID_POLE_TO_STOW" : "ArmMovements.STOW_TO_MID_POLE";
                    }
                    case LOW -> {
                        scoreCommand = ifReversed ? ArmMovements.GROUND_TO_STOW : ArmMovements.STOW_TO_GROUND;
                        scoreCommandName = ifReversed ? "ArmMovements.GROUND_TO_STOW" : "ArmMovements.STOW_TO_GROUND";
                    }
                }
            }
            case CUBE -> {
                if (ifCone) mode = ConeOrCube.CONE;
                switch (level) {
                    case HIGH -> {
                        scoreCommand = ifReversed ? ArmMovements.HIGH_BOX_TO_STOW : ArmMovements.STOW_TO_HIGH_BOX;
                        scoreCommandName = ifReversed ? "ArmMovements.HIGH_BOX_TO_STOW" : "ArmMovements.STOW_TO_HIGH_BOX";
                    }
                    case MID -> {
                        scoreCommand = ifReversed ? ArmMovements.MID_BOX_TO_STOW : ArmMovements.STOW_TO_MID_BOX;
                        scoreCommandName = ifReversed ? "ArmMovements.MID_BOX_TO_STOW" : "ArmMovements.STOW_TO_MID_BOX";
                    }
                    case LOW -> {
                        scoreCommand = ifReversed ? ArmMovements.GROUND_TO_STOW : ArmMovements.STOW_TO_GROUND;
                        scoreCommandName = ifReversed ? "ArmMovements.GROUND_TO_STOW" : "ArmMovements.STOW_TO_GROUND";
                    }
                }
            }
        }
        collectUpCommand = ifUpCollectReversed ? ArmMovements.UP_COLLECT_TO_STOW : ArmMovements.STOW_TO_UP_COLLECT;
        upName = ifUpCollectReversed ? "ArmMovements.UP_COLLECT_TO_STOW" : "ArmMovements.STOW_TO_UP_COLLECT";
        collectDownCommand = ifDownCollectReversed ? ArmMovements.DOWN_COLLECT_TO_STOW : ArmMovements.STOW_TO_DOWN_COLLECT;
        downName = ifDownCollectReversed ? "ArmMovements.DOWN_COLLECT_TO_STOW" : "ArmMovements.STOW_TO_DOWN_COLLECT";
        SmartDashboard.putString("CONE OR CUBE", mode.toString());
        SmartDashboard.putString("LEVEL SELECT", level.toString());
        SmartDashboard.putString("SCORE COMMAND", scoreCommandName);
        SmartDashboard.putString("UP COMMAND", upName);
        SmartDashboard.putString("DOWN COMMAND", downName);
        SmartDashboard.putBoolean("CONE?", ifCone);
        SmartDashboard.putBoolean("REVERSED?", ifReversed);
    }

    private void scheduleArmCommand(ArmMoveCommand armCommand) {
        armCommand.schedule();
    }
    private void scheduleCollectCommand(ArmMoveCommand collectCommand) {
        collectCommand.schedule();
    }

    private void resetClimb() {
        resetCommand.schedule();
    }

    private void initiateClimb() {
        climbSequence.schedule();
    }

    private static final double DEAD_ZONE = 0.09;
    //If 9% does not fell responsive enough try 10.5%

    private static double deadband(double value) {
        if (Math.abs(value) > OI.DEAD_ZONE) {
            if (value > 0.0) {
                return (value - OI.DEAD_ZONE) / (1.0 - OI.DEAD_ZONE);
            } else {
                return (value + OI.DEAD_ZONE) / (1.0 - OI.DEAD_ZONE);
            }
        } else {
            return 0.0;
        }
    }

    /**
     * Reduces the sensitivity around the zero point to make the Robot more
     * controllable.
     *
     * @param value - raw input
     * @param power - 1.0 indicates linear (full sensitivity) - larger number
     *              reduces small values
     *
     * @return 0 to +/- 100%
     */
    private double desensitizePowerBased(double value, double power) {
        value = deadband(value);
        return Math.pow(Math.abs(value), power - 1) * value;
    }
}
