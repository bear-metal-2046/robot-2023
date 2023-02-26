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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.tahomarobotics.robot.wrist.WristPosition;

import java.util.List;

public class ArmMovements {

    //Trajectory Configurations
    private static final TrajectoryConfig NORMAL_SPEED =
            new TrajectoryConfig(3, 3);
    private static final TrajectoryConfig SLOW_SPEED =
            new TrajectoryConfig(1, 1);

    //Start, Stow, and Collect positions
    private static final Translation2d START = new Translation2d(Units.inchesToMeters(22.8), Units.inchesToMeters(-4));
    private static final Translation2d STOW = new Translation2d(Units.inchesToMeters(15.1), Units.inchesToMeters(3));
    private static final Translation2d DOWN_COLLECT = new Translation2d(Units.inchesToMeters(30), Units.inchesToMeters(-11));
    private static final Translation2d UP_COLLECT = new Translation2d(Units.inchesToMeters(26.1), Units.inchesToMeters(-6.8));
    private static final Translation2d FEEDER_COLLECT = new Translation2d(Units.inchesToMeters(21.5), Units.inchesToMeters(5.1));

    //Box scoring positions
    private static final Translation2d MID_BOX = new Translation2d(Units.inchesToMeters(41.8), Units.inchesToMeters(19.2));
    private static final Translation2d HIGH_BOX = new Translation2d(Units.inchesToMeters(54), Units.inchesToMeters(30));

    //Pole scoring positions
    private static final Translation2d MID_POLE = new Translation2d(Units.inchesToMeters(44.8), Units.inchesToMeters(29.3));
    private static final Translation2d HIGH_POLE = new Translation2d(Units.inchesToMeters(53.7), Units.inchesToMeters(36.6));

    //Translations
    private static final List<Translation2d> NONE = List.of();

    //Rotations
    private static final Rotation2d UP = new Rotation2d(Units.degreesToRadians(90));
    private static final Rotation2d DOWN = new Rotation2d(Units.degreesToRadians(-90));
    private static final Rotation2d FWD = new Rotation2d(Units.degreesToRadians(0));
    private static final Rotation2d REV = new Rotation2d(Units.degreesToRadians(180));

    //Commands
    public static final ArmMoveCommand START_TO_STOW =
            new ArmMoveCommand("Start to Stow", new Pose2d(START, UP), NONE, new Pose2d(STOW, UP), SLOW_SPEED,
                    WristPosition.STOW);

    public static final InstantCommand POSITION_TO_STOW_COMMAND = new InstantCommand(()->
        new ArmMoveCommand("Stow", Arm.getInstance().getCurrentPosition(), STOW, SLOW_SPEED, WristPosition.STOW).schedule());

    public static final ArmMoveCommand STOW_TO_DOWN_COLLECT =
            new ArmMoveCommand("Down Collect", new Pose2d(STOW, FWD), NONE, new Pose2d(DOWN_COLLECT, DOWN), NORMAL_SPEED,
                    WristPosition.DOWN_COLLECT);
    public static final ArmMoveCommand STOW_TO_UP_COLLECT =
            new ArmMoveCommand("Up Collect", new Pose2d(STOW, FWD), NONE, new Pose2d(UP_COLLECT, DOWN), NORMAL_SPEED,
                    WristPosition.UP_COLLECT);
    public static final ArmMoveCommand STOW_TO_FEEDER_COLLECT =
            new ArmMoveCommand("Feeder Collect", STOW, FEEDER_COLLECT, NORMAL_SPEED,
                    WristPosition.FEEDER_COLLECT);
    public static final ArmMoveCommand STOW_TO_MID_BOX =
            new ArmMoveCommand("Mid-Box", STOW, MID_BOX, NORMAL_SPEED,
                    WristPosition.MIDBOXPLACE);
    public static final ArmMoveCommand STOW_TO_HIGH_BOX =
            new ArmMoveCommand("High-Box", STOW, HIGH_BOX, NORMAL_SPEED,
                    WristPosition.HIGHBOXPLACE);
    public static final ArmMoveCommand STOW_TO_MID_POLE =
            new ArmMoveCommand("Mid-Pole", STOW, MID_POLE, NORMAL_SPEED,
                    WristPosition.MIDPOLEPLACE);
    public static final ArmMoveCommand STOW_TO_HIGH_POLE =
            new ArmMoveCommand("High-Pole", STOW, HIGH_POLE, NORMAL_SPEED,
                    WristPosition.HIGHPOLEPLACE);

    //Reversed Commands
    public static final ArmMoveCommand UP_COLLECT_TO_STOW =
            new ArmMoveCommand("Up Collect to Stow", new Pose2d(UP_COLLECT, UP), NONE, new Pose2d(STOW, REV), NORMAL_SPEED,
                    WristPosition.STOW);
    public static final ArmMoveCommand DOWN_COLLECT_TO_STOW =
            new ArmMoveCommand("Down Collect Stow", new Pose2d(DOWN_COLLECT, UP), NONE, new Pose2d(STOW, REV), NORMAL_SPEED,
                    WristPosition.STOW);
    public static final ArmMoveCommand FEEDER_COLLECT_TO_STOW =
            new ArmMoveCommand("Feeder Collect", FEEDER_COLLECT, STOW, NORMAL_SPEED,
                    WristPosition.STOW);
    public static final ArmMoveCommand MID_BOX_TO_STOW =
            new ArmMoveCommand("Mid-Box Stow", MID_BOX, STOW, NORMAL_SPEED,
                    WristPosition.STOW);
    public static final ArmMoveCommand HIGH_BOX_TO_STOW =
            new ArmMoveCommand("High-Box Stow", HIGH_BOX, STOW, NORMAL_SPEED,
                    WristPosition.STOW);
    public static final ArmMoveCommand MID_POLE_TO_STOW =
            new ArmMoveCommand("Mid-Pole Stow", MID_POLE, STOW, NORMAL_SPEED,
                    WristPosition.STOW);
    public static final ArmMoveCommand HIGH_POLE_TO_STOW =
            new ArmMoveCommand("High-Pole Stow", HIGH_POLE, STOW, NORMAL_SPEED,
                    WristPosition.STOW);

}
