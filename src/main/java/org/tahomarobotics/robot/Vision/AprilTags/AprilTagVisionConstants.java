package org.tahomarobotics.robot.Vision.AprilTags;

import edu.wpi.first.math.util.Units;

@Deprecated(since = "From 2022; Please Update")
public class AprilTagVisionConstants {
    public static final double CAMERA_ANGLE = Units.degreesToRadians(34.3213); // Adjusted base on results
    public static final double CAMERA_HEIGHT = Units.inchesToMeters(27.7487); // Adjusted based on result
    private static final double TAPE_HEIGHT = Units.inchesToMeters(102.369); // From field CAD
    public static final double HEIGHT_DIFFERENCE = TAPE_HEIGHT - CAMERA_HEIGHT + Units.inchesToMeters(-1.3);
    public static final double CAMERA_TO_CENTER = Units.inchesToMeters(10);
}
