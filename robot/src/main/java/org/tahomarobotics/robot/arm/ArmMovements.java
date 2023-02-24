package org.tahomarobotics.robot.arm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;

import java.util.List;

public class ArmMovements {

    //Trajectory Configurations
    private static final TrajectoryConfig SLOW_TEST_CFG =
            new TrajectoryConfig(3, 3);
    private static final TrajectoryConfig REVERSED_SLOW_TEST =
            new TrajectoryConfig(3, 3).setReversed(true);

    //Start and stow positions
    private static final Translation2d START = new Translation2d(Units.inchesToMeters(22.8), Units.inchesToMeters(-4));
    private static final Translation2d STOW = new Translation2d(Units.inchesToMeters(15.1), Units.inchesToMeters(3));

    //Ground scoring position
    private static final Translation2d GROUND_SCORE = new Translation2d(Units.inchesToMeters(34.33), Units.inchesToMeters(-0.1445));

    //Box scoring positions
    private static final Translation2d MID_BOX = new Translation2d(Units.inchesToMeters(46.32), Units.inchesToMeters(21.07));
    private static final Translation2d HIGH_BOX = new Translation2d(Units.inchesToMeters(57.94), Units.inchesToMeters(28.113));

    //Pole scoring positions
    private static final Translation2d MID_POLE = new Translation2d(Units.inchesToMeters(50.612), Units.inchesToMeters(28.87));
    private static final Translation2d HIGH_POLE = new Translation2d(Units.inchesToMeters(54.8), Units.inchesToMeters(35.5));

    //Translations
    private static final List<Translation2d> NONE = List.of();

    //Rotations
    private static final Rotation2d UP = new Rotation2d(Units.degreesToRadians(90));
    private static final Rotation2d DOWN = new Rotation2d(Units.degreesToRadians(-90));
    private static final Rotation2d FWD = new Rotation2d(Units.degreesToRadians(0));
    private static final Rotation2d REV = new Rotation2d(Units.degreesToRadians(180));

    //Commands
    public static final ArmMoveCommand START_TO_STOW = new ArmMoveCommand(new Pose2d(START, UP), NONE, new Pose2d(STOW, UP), SLOW_TEST_CFG);
    public static final ArmMoveCommand STOW_TO_GROUND = new ArmMoveCommand(new Pose2d(STOW, UP), NONE, new Pose2d(GROUND_SCORE, FWD), SLOW_TEST_CFG);
    public static final ArmMoveCommand STOW_TO_MID_BOX = new ArmMoveCommand(new Pose2d(STOW, UP), NONE, new Pose2d(MID_BOX, FWD), SLOW_TEST_CFG);
    public static final ArmMoveCommand STOW_TO_HIGH_BOX = new ArmMoveCommand(new Pose2d(STOW, UP), NONE, new Pose2d(HIGH_BOX, FWD), SLOW_TEST_CFG);
    public static final ArmMoveCommand STOW_TO_MID_POLE = new ArmMoveCommand(new Pose2d(STOW, UP), NONE, new Pose2d(MID_POLE, FWD), SLOW_TEST_CFG);
    public static final ArmMoveCommand STOW_TO_HIGH_POLE = new ArmMoveCommand(new Pose2d(STOW, UP), NONE, new Pose2d(HIGH_POLE, FWD), SLOW_TEST_CFG);

    //Reversed Commands
    public static final ArmMoveCommand STOW_TO_START = new ArmMoveCommand(new Pose2d(START, UP), NONE, new Pose2d(STOW, UP), REVERSED_SLOW_TEST);
    public static final ArmMoveCommand GROUND_TO_STOW = new ArmMoveCommand(new Pose2d(GROUND_SCORE, FWD), NONE, new Pose2d(STOW, UP), REVERSED_SLOW_TEST);
    public static final ArmMoveCommand MID_BOX_TO_STOW = new ArmMoveCommand(new Pose2d(MID_BOX, FWD), NONE, new Pose2d(STOW, UP), REVERSED_SLOW_TEST);
    public static final ArmMoveCommand HIGH_BOX_TO_STOW = new ArmMoveCommand(new Pose2d(HIGH_BOX, FWD), NONE, new Pose2d(STOW, UP), REVERSED_SLOW_TEST);
    public static final ArmMoveCommand MID_POLE_TO_STOW = new ArmMoveCommand(new Pose2d(MID_POLE, FWD), NONE, new Pose2d(STOW, UP), REVERSED_SLOW_TEST);
    public static final ArmMoveCommand HIGH_POLE_TO_STOW = new ArmMoveCommand(new Pose2d(HIGH_POLE, FWD), NONE, new Pose2d(STOW, UP), REVERSED_SLOW_TEST);

    /*

    Ground level score - (33.62, -8.98)

    Box levels:
    Mid level score - (49.43, 8.77)
    High level score - (69.3, 20.74)

    Cone levels: (at top of pole)
    Mid pole - (29.18, 13.25)
    High pole - (66.21, 25.25)

     */
}
