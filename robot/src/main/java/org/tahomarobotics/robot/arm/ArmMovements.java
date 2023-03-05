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
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.wrist.WristPosition;

import java.util.List;

public class ArmMovements {

    private static final Logger logger = LoggerFactory.getLogger(ArmMovements.class);

    private static final double MIN_DISTANCE = Units.inchesToMeters(4);

    //Trajectory Configurations
    private static final TrajectoryConfig NORMAL_SPEED =
            new TrajectoryConfig(3, 3);
    private static final TrajectoryConfig SLOW_SPEED =
            new TrajectoryConfig(0.5, 1);


    private static final Translation2d START = new Translation2d(Units.inchesToMeters(22.8), Units.inchesToMeters(-4));
    static final Translation2d STOW = new Translation2d(Units.inchesToMeters(16.2), Units.inchesToMeters(-1.2));

    //Collecting
    private static final Translation2d CUBE_COLLECT = new Translation2d(Units.inchesToMeters(28.4), Units.inchesToMeters(-12.0));
    private static final Translation2d CONE_COLLECT = new Translation2d(Units.inchesToMeters(28.4), Units.inchesToMeters(-12.0));
    private static final Translation2d CONE_FEEDER_COLLECT = new Translation2d(Units.inchesToMeters(19.0), Units.inchesToMeters(4.2));
    private static final Translation2d CUBE_FEEDER_COLLECT = new Translation2d(Units.inchesToMeters(19.0), Units.inchesToMeters(4.2));

    //Box scoring positions
    private static final Translation2d MID_BOX = new Translation2d(Units.inchesToMeters(41.0), Units.inchesToMeters(19.4));
    private static final Translation2d HIGH_BOX = new Translation2d(Units.inchesToMeters(52.8), Units.inchesToMeters(29.5));

    //Pole scoring positions
    private static final Translation2d MID_POLE = new Translation2d(Units.inchesToMeters(45.2), Units.inchesToMeters(27.8));
    private static final Translation2d HIGH_POLE = new Translation2d(Units.inchesToMeters(54.2), Units.inchesToMeters(36));

    //Translations
    private static final List<Translation2d> NONE = List.of();

    //Rotations
    private static final Rotation2d UP = new Rotation2d(Units.degreesToRadians(90));
    private static final Rotation2d DOWN = new Rotation2d(Units.degreesToRadians(-90));
    private static final Rotation2d FWD = new Rotation2d(Units.degreesToRadians(0));
    private static final Rotation2d REV = new Rotation2d(Units.degreesToRadians(180));

    public record ArmMove(String name, ArmTrajectory trajectory, WristPosition wristPosition) {}

    public static final ArmMove START_TO_STOW = new ArmMove("Start to Stow",
            new ArmTrajectory(new Pose2d(START, UP), NONE, new Pose2d(STOW, UP), SLOW_SPEED),
            WristPosition.STOW);

    public static ArmMoveCommand createPositionToStowCommand() {
        return new ArmMoveCommand("Pos To Stow", createPositionToStowTrajectory(Arm.getInstance().getCurrentPosition()), WristPosition.STOW);
    }
    static ArmTrajectory createPositionToStowTrajectory(Translation2d position) {
        Translation2d delta = position.minus(STOW);
        double norm = delta.getNorm();
        if (norm < MIN_DISTANCE) {

            double scalar = MIN_DISTANCE/norm;
            delta = delta.times(scalar);
            Translation2d moved = STOW.plus(delta);

            logger.warn("Moving position from " + position + " to " + moved);
            position = moved;
        }

        return new ArmTrajectory(position, STOW, SLOW_SPEED);
    }

    public static final ArmMove STOW_TO_CUBE_COLLECT = new ArmMove("Cube Collect",
            new ArmTrajectory(new Pose2d(STOW, FWD), NONE, new Pose2d(CUBE_COLLECT, DOWN), NORMAL_SPEED),
            WristPosition.CUBE_COLLECT);

    public static final ArmMove STOW_TO_CONE_COLLECT = new ArmMove("Cone Collect",
            new ArmTrajectory(new Pose2d(STOW, FWD), NONE, new Pose2d(CONE_COLLECT, DOWN), NORMAL_SPEED),
            WristPosition.CONE_COLLECT);

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
            new ArmTrajectory(STOW, HIGH_POLE, NORMAL_SPEED),
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

    public static final ArmMove MID_BOX_TO_STOW = new ArmMove("Mid-Box Stow",
            new ArmTrajectory(MID_BOX, STOW, NORMAL_SPEED),
            WristPosition.STOW);

    public static final ArmMove HIGH_BOX_TO_STOW = new ArmMove("High-Box Stow",
            new ArmTrajectory(HIGH_BOX, STOW, NORMAL_SPEED),
            WristPosition.STOW);

    public static final ArmMove MID_POLE_TO_STOW = new ArmMove("Mid-Pole Stow",
            new ArmTrajectory(MID_POLE, STOW, NORMAL_SPEED),
            WristPosition.STOW);

    public static final ArmMove HIGH_POLE_TO_STOW = new ArmMove("High-Pole Stow",
            new ArmTrajectory(new Pose2d(HIGH_POLE, REV), NONE, new Pose2d(STOW, DOWN), NORMAL_SPEED),
            WristPosition.STOW);

}