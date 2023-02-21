package org.tahomarobotics.robot.arm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;

import java.util.List;

public class ArmMovements {
    private static final TrajectoryConfig TRAJ_CFG = new TrajectoryConfig(0.25, 0.5);
    private static final TrajectoryConfig SLOW_CLIMB_CFG = new TrajectoryConfig(0.5, 1.0);
    private static final TrajectoryConfig FAST_CLIMB_CFG = new TrajectoryConfig(3.0, 4.0);

    private static final Translation2d CLIMB_OUT = new Translation2d(Units.inchesToMeters(61.96), Units.inchesToMeters(17.29));
    private static final Translation2d CLIMB_TOP = new Translation2d(Units.inchesToMeters(8.6), Units.inchesToMeters(16));
    private static final Translation2d CLIMB_IN = new Translation2d(Units.inchesToMeters(35.96), Units.inchesToMeters(17.29));
    private static final Translation2d START = new Translation2d(Units.inchesToMeters(22.8), Units.inchesToMeters(-4));
    private static final Translation2d STOW = new Translation2d(Units.inchesToMeters(15.1), Units.inchesToMeters(3));
    private static final Translation2d HIGH = new Translation2d(Units.inchesToMeters(48), Units.inchesToMeters(24));

    private static final List<Translation2d> CLIMB_SWING_MID = List.of(CLIMB_TOP);
    private static final List<Translation2d> NONE = List.of();

    private static final Rotation2d UP = new Rotation2d(Units.degreesToRadians(90));
    private static final Rotation2d DOWN = new Rotation2d(Units.degreesToRadians(-90));
    private static final Rotation2d FWD = new Rotation2d(Units.degreesToRadians(0));
    private static final Rotation2d REV = new Rotation2d(Units.degreesToRadians(180));

    public static final ArmMoveCommand CLIMB_SWING_COMMAND_OUT = new ArmMoveCommand(new Pose2d(STOW, FWD), NONE, new Pose2d(CLIMB_OUT, UP), SLOW_CLIMB_CFG);
    public static final ArmMoveCommand CLIMB_SWING_COMMAND_IN = new ArmMoveCommand(new Pose2d(CLIMB_OUT, FWD), CLIMB_SWING_MID, new Pose2d(CLIMB_IN, FWD), FAST_CLIMB_CFG);
//    public static final ArmMoveCommand PIECE_PICK_UP_COMMAND = new ArmMoveCommand(new Pose2d(STOW, FWD), NONE, new Pose2d(new Translation2d(Units.inchesToMeters(52.4), Units.inchesToMeters(-7.6)), REV), TRAJ_CFG);
    public static final ArmMoveCommand START_TO_STOW_ARM_COMMAND = new ArmMoveCommand(new Pose2d(START, UP), NONE, new Pose2d(STOW, UP), TRAJ_CFG);
    public static final ArmMoveCommand HIGH_TO_STOW_ARM_COMMAND = new ArmMoveCommand(new Pose2d(HIGH, REV), NONE, new Pose2d(STOW, REV), TRAJ_CFG);
    public static final ArmMoveCommand STOW_TO_HIGH_ARM_COMMAND = new ArmMoveCommand(new Pose2d(STOW, FWD), NONE, new Pose2d(HIGH, FWD), TRAJ_CFG);
}
