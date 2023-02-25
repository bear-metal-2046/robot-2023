package org.tahomarobotics.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class StartPoseTest {
    @Test
    void testFromPose(){
        Pose2d testLeftPose = new Pose2d(Units.inchesToMeters(70), Units.inchesToMeters(35), new Rotation2d(0));

        assertEquals(StartPose.LEFT,StartPose.fromPose(testLeftPose));

        Pose2d testMidPose = new Pose2d(Units.inchesToMeters(75), Units.inchesToMeters(110), new Rotation2d(0));

        assertEquals(StartPose.MID,StartPose.fromPose(testMidPose));

        Pose2d testRightPose = new Pose2d(Units.inchesToMeters(80), Units.inchesToMeters(175), new Rotation2d(0));

        assertEquals(StartPose.RIGHT,StartPose.fromPose(testRightPose));


    }
}
