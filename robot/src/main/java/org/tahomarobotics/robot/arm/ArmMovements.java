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
package org.tahomarobotics.robot.arm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.wrist.WristMoveCommand;
import org.tahomarobotics.robot.wrist.WristPosition;

import java.util.List;

public class ArmMovements {

    private static final Logger logger = LoggerFactory.getLogger(ArmMovements.class);

    private static final double MIN_DISTANCE = Units.inchesToMeters(4);

    //Trajectory Configurations
    private static final TrajectoryConfig NORMAL_SPEED =
            new TrajectoryConfig(4, 4);
    private static final TrajectoryConfig SLOW_SPEED =
            new TrajectoryConfig(0.5, 1);


    private static final Translation2d START = new Translation2d(Units.inchesToMeters(10.5), Units.inchesToMeters(-2.1));
    static final Translation2d STOW = new Translation2d(Units.inchesToMeters(11.0), Units.inchesToMeters(0.0));
    static final Translation2d PRE_CLIMB = new Translation2d(Units.inchesToMeters(22.7), Units.inchesToMeters(4.7));

    //Collecting
    private static final Translation2d CUBE_COLLECT = new Translation2d(Units.inchesToMeters(22.5), Units.inchesToMeters(-10.5));
    private static final Translation2d CONE_COLLECT = new Translation2d(Units.inchesToMeters(22.5), Units.inchesToMeters(-10.5));
    private static final Translation2d CONE_FEEDER_COLLECT = new Translation2d(Units.inchesToMeters(15.75), Units.inchesToMeters(9.4));
    private static final Translation2d CUBE_FEEDER_COLLECT = new Translation2d(Units.inchesToMeters(15.75), Units.inchesToMeters(9.4));
    private static final Translation2d CUBE_SLIDER_COLLECT = new Translation2d(Units.inchesToMeters(24.2), Units.inchesToMeters(28.1));
    private static final Translation2d CONE_SLIDER_COLLECT = new Translation2d(Units.inchesToMeters(24.2), Units.inchesToMeters(27.85));


    //Box scoring positions
    private static final Translation2d MID_BOX = new Translation2d(Units.inchesToMeters(33.8), Units.inchesToMeters(16.1));
    private static final Translation2d HIGH_BOX = new Translation2d(Units.inchesToMeters(54.4), Units.inchesToMeters(30.6));

    //Pole scoring positions
    private static final Translation2d MID_POLE = new Translation2d(Units.inchesToMeters(32.5), Units.inchesToMeters(20.0));
    private static final Translation2d HIGH_POLE = new Translation2d(Units.inchesToMeters(50.25), Units.inchesToMeters(29.5));
    private static final Translation2d START_TO_HIGH_MID_PT = new Translation2d(Units.inchesToMeters(25.51), Units.inchesToMeters(10.91));

    //Translations
    private static final List<Translation2d> NONE = List.of();

    //Rotations
    private static final Rotation2d UP = new Rotation2d(Units.degreesToRadians(90));
    private static final Rotation2d DOWN = new Rotation2d(Units.degreesToRadians(-90));
    private static final Rotation2d FWD = new Rotation2d(Units.degreesToRadians(0));
    private static final Rotation2d REV = new Rotation2d(Units.degreesToRadians(180));

    public record ArmMove(String name, ArmTrajectory trajectory, WristPosition wristPosition) {
        public CommandBase createArmWristMoveCommand() {
            CommandBase cmd = new ArmMoveCommand(name, trajectory);

            if (trajectory.isValid()) {
                var total = trajectory.getTotalTimeSeconds();
                var waitTime = 0.25 * total;
                var wristTime = 0.5 * total;
                cmd = cmd.alongWith(new WaitCommand(waitTime).andThen(new WristMoveCommand(wristPosition, wristTime)));
            }
            return cmd;
        }
    }

//    private static ArmTrajectory CLIMB_SWING_TRAJ = ClimbSwingGenerator.generateTrajectory(
//            5, 0.5, 0.15, 0.1, Units.degreesToRadians(-138.5),
//            Units.degreesToRadians(76.4), Units.degreesToRadians(118), Units.degreesToRadians(138));
    private static ArmTrajectory CLIMB_SWING_TRAJ = ClimbSwingGenerator.generateTrajectory(
            4.5, 0.5, 0.15, 0.15, Units.degreesToRadians(-138.5),
            Units.degreesToRadians(76.4), Units.degreesToRadians(118), Units.degreesToRadians(138));
    public static final ArmMove STOW_TO_CLIMB = new ArmMove("Stow To Climb",
            new ArmTrajectory(STOW, PRE_CLIMB, SLOW_SPEED),
            WristPosition.PRE_CLIMB);

    public static ArmMove CLIMB_SWING = new ArmMove("Climb Swing", CLIMB_SWING_TRAJ, WristPosition.PRE_CLIMB);
    public static final ArmMove START_TO_STOW = new ArmMove("Start to Stow",
            new ArmTrajectory(new Pose2d(START, UP), NONE, new Pose2d(STOW, UP), SLOW_SPEED),
            WristPosition.STOW);
    public static final ArmMove START_TO_HIGH_POLE = new ArmMove("Start to High-Pole",
            new ArmTrajectory(new Pose2d(START, UP), List.of(START_TO_HIGH_MID_PT), new Pose2d(HIGH_POLE, FWD), NORMAL_SPEED),
            WristPosition.HIGH_POLE_PLACE);

    public static ProxyCommand createPositionToStowCommand() {
        return new ProxyCommand(() -> new ArmMove("Pos To Stow",
                createPositionToStowTrajectory(Arm.getInstance().getCurrentPosition(), STOW), WristPosition.STOW)
                .createArmWristMoveCommand());
    }
    public static ArmTrajectory createPositionToStowTrajectory(Translation2d position, Translation2d desired) {
        Translation2d delta = position.minus(desired);
        double norm = delta.getNorm();
        if (norm < MIN_DISTANCE) {

            double scalar = MIN_DISTANCE/norm;
            delta = delta.times(scalar);
            Translation2d moved = desired.plus(delta);

            logger.warn("Moving position from " + position + " to " + moved);
            position = moved;
        }

        return new ArmTrajectory(position, desired, SLOW_SPEED);
    }

    public static final ArmMove STOW_TO_CUBE_COLLECT = new ArmMove("Cube Collect",
            new ArmTrajectory(new Pose2d(STOW, FWD), NONE, new Pose2d(CUBE_COLLECT, DOWN), NORMAL_SPEED),
            WristPosition.CUBE_COLLECT);

    public static final ArmMove STOW_TO_CONE_COLLECT = new ArmMove("Cone Collect",
            new ArmTrajectory(new Pose2d(STOW, FWD), NONE, new Pose2d(CONE_COLLECT, DOWN), NORMAL_SPEED),
            WristPosition.CONE_COLLECT);

    public static final ArmMove STOW_TO_CONE_SLIDER_COLLECT = new ArmMove("Slider Collect",
            new ArmTrajectory(new Pose2d(STOW, FWD), NONE, new Pose2d(CONE_SLIDER_COLLECT, DOWN), NORMAL_SPEED),
            WristPosition.CONE_SLIDER_COLLECT);
    public static final ArmMove STOW_TO_CUBE_SLIDER_COLLECT = new ArmMove("Slider Collect",
            new ArmTrajectory(new Pose2d(STOW, FWD), NONE, new Pose2d(CUBE_SLIDER_COLLECT, DOWN), NORMAL_SPEED),
            WristPosition.CUBE_SLIDER_COLLECT);

    public static final ArmMove STOW_TO_CONE_FEEDER_COLLECT = new ArmMove("Feeder Collect",
            new ArmTrajectory(STOW, CONE_FEEDER_COLLECT, NORMAL_SPEED),
            WristPosition.CONE_FEEDER_COLLECT);

    public static final ArmMove STOW_TO_CUBE_FEEDER_COLLECT = new ArmMove("Feeder Collect",
            new ArmTrajectory(STOW, CUBE_FEEDER_COLLECT, NORMAL_SPEED),
            WristPosition.CUBE_FEEDER_COLLECT);

    public static final ArmMove STOW_TO_MID_BOX = new ArmMove("Mid-Box",
            new ArmTrajectory(STOW, MID_BOX, NORMAL_SPEED),
            WristPosition.MID_BOX_PLACE);

    public static final ArmMove STOW_TO_HIGH_BOX = new ArmMove("High-Box",
            new ArmTrajectory(STOW, HIGH_BOX, NORMAL_SPEED),
            WristPosition.HIGH_BOX_PLACE);

    public static final ArmMove STOW_TO_MID_POLE = new ArmMove("Mid-Pole",
            new ArmTrajectory(STOW, MID_POLE, NORMAL_SPEED),
        WristPosition.MID_POLE_PLACE);

    public static final ArmMove STOW_TO_HIGH_POLE = new ArmMove("High-Pole",
            new ArmTrajectory(new Pose2d(STOW, UP), NONE, new Pose2d(HIGH_POLE, FWD), NORMAL_SPEED),
            WristPosition.HIGH_POLE_PLACE);

    //Reversed Commands
    public static final ArmMove CONE_COLLECT_TO_STOW = new ArmMove("Cone Collect Stow",
            new ArmTrajectory(new Pose2d(CONE_COLLECT, UP), NONE, new Pose2d(STOW, REV), NORMAL_SPEED),
            WristPosition.STOW);

    public static final ArmMove CUBE_COLLECT_TO_STOW = new ArmMove("Cube Collect Stow",
            new ArmTrajectory(new Pose2d(CUBE_COLLECT, UP), NONE, new Pose2d(STOW, REV), NORMAL_SPEED),
            WristPosition.STOW);

    public static final ArmMove CONE_FEEDER_COLLECT_TO_STOW = new ArmMove("Feeder Collect Stow",
            new ArmTrajectory(CONE_FEEDER_COLLECT, STOW, NORMAL_SPEED),
            WristPosition.STOW);

    public static final ArmMove CUBE_FEEDER_COLLECT_TO_STOW = new ArmMove("Feeder Collect Stow",
            new ArmTrajectory(CUBE_FEEDER_COLLECT, STOW, NORMAL_SPEED),
            WristPosition.STOW);

    public static final ArmMove CONE_SLIDER_COLLECT_TO_STOW = new ArmMove("Slider Collect Stow",
            new ArmTrajectory(CONE_SLIDER_COLLECT, STOW, NORMAL_SPEED),
            WristPosition.STOW);

    public static final ArmMove CUBE_SLIDER_COLLECT_TO_STOW = new ArmMove("Slider Collect Stow",
            new ArmTrajectory(CUBE_SLIDER_COLLECT, STOW, NORMAL_SPEED),
            WristPosition.STOW);

    public static final ArmMove MID_BOX_TO_STOW = new ArmMove("Mid-Box Stow",
            new ArmTrajectory(MID_BOX, STOW, NORMAL_SPEED),
            WristPosition.STOW);

    public static final ArmMove HIGH_BOX_TO_STOW = new ArmMove("High-Box Stow",
            new ArmTrajectory(HIGH_BOX, STOW, NORMAL_SPEED),
            WristPosition.STOW);

    public static final ArmMove HIGH_BOX_TO_CUBE_COLLECT = new ArmMove("High-Box Cube",
            new ArmTrajectory(new Pose2d(HIGH_BOX, REV), NONE, new Pose2d(CUBE_COLLECT, DOWN), NORMAL_SPEED),
            WristPosition.STOW);

    public static final ArmMove MID_POLE_TO_STOW = new ArmMove("Mid-Pole Stow",
            new ArmTrajectory(MID_POLE, STOW, NORMAL_SPEED),
            WristPosition.STOW);

    public static final ArmMove HIGH_POLE_TO_STOW = new ArmMove("High-Pole Stow",
            new ArmTrajectory(new Pose2d(HIGH_POLE, REV), NONE, new Pose2d(STOW, DOWN), NORMAL_SPEED),
            WristPosition.STOW);

}