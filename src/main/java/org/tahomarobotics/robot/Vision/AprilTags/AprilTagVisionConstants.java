package org.tahomarobotics.robot.Vision.AprilTags;

import edu.wpi.first.math.util.Units;

@Deprecated(since = "From 2022; Please Update")
public class AprilTagVisionConstants {
    //TODO Adjust according to design specifications once acquired
    public static final double CAMERA_ANGLE = Units.degreesToRadians(34.3213); // Adjusted base on results
    //TODO Adjust according to design specifications once acquired.
    public static final double CAMERA_HEIGHT = Units.inchesToMeters(27.7487); // Adjusted based on result

    //TODO This var shouldn't be needed anymore, can likely be removed. AT translation can provide height on its own generally.
    private static final double TAPE_HEIGHT = Units.inchesToMeters(102.369); // From field CAD
    public static final double HEIGHT_DIFFERENCE = TAPE_HEIGHT - CAMERA_HEIGHT + Units.inchesToMeters(-1.3);

    //TODO Adjust according to design specifications once acquired.
    public static final double CAMERA_TO_CENTER = Units.inchesToMeters(10);
}
