package org.tahomarobotics.robot.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    public static final Transform3d FRONT_CAM_OFFSET = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(-3.514), Units.inchesToMeters(-9.638), Units.inchesToMeters(23.616)
        ),
        new Rotation3d(
            0, 0, Units.degreesToRadians(-15)
        )
    );

    public static final Transform3d BACK_CAM_OFFSET = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(-13.514), Units.inchesToMeters(-6.545), Units.inchesToMeters(23.616)
        ),
        new Rotation3d(
            0, 0, Units.degreesToRadians(180)
        )
    );

    public static final double POSE_DIFFERENCE_THRESHOLD = 2;
    public static final double DEGREES_DIFFERENCE_THRESHOLD = 60;
    public static final double TARGET_DISTANCE_THRESHOLD = 10;
    public static final double SINGLE_TARGET_DISTANCE_THRESHOLD = 4;
    public static final int LARGE_DIFFERENCE_COUNT_ACCEPTANCE = 3;
}
