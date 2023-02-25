package org.tahomarobotics.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import org.junit.jupiter.api.Test;
import org.mockito.MockedStatic;
import org.mockito.Mockito;
import org.tahomarobotics.robot.auto.AllianceUtil;

import static org.junit.jupiter.api.Assertions.*;

class AllianceUtilTest {

    @Test
    void adjustPoseForAlliance() {
        try (MockedStatic<DriverStation> ds = Mockito.mockStatic(DriverStation.class)) {
            ds.when(DriverStation::getAlliance).thenReturn(DriverStation.Alliance.Red);
            Pose2d startPose = new Pose2d(10, 10, new Rotation2d(-Math.PI / 4));
            Pose2d expectedPose = new Pose2d(641.25, 10, new Rotation2d(-Math.PI * 3 / 4));

            Pose2d adjustedPose = AllianceUtil.adjustPoseForAlliance(startPose);
            assertEquals(startPose, AllianceUtil.adjustPoseForAlliance(adjustedPose));

            startPose = new Pose2d(10, 10, new Rotation2d(0));
            expectedPose = new Pose2d(641.25, 10, new Rotation2d(Math.PI));
            adjustedPose = AllianceUtil.adjustPoseForAlliance(startPose);
            assertEquals(startPose, AllianceUtil.adjustPoseForAlliance(adjustedPose));

            startPose = new Pose2d(10, 10, new Rotation2d(Math.PI / 2));
            expectedPose = new Pose2d(641.25, 10, new Rotation2d(Math.PI / 2));
            adjustedPose = AllianceUtil.adjustPoseForAlliance(startPose);
            assertEquals(startPose, AllianceUtil.adjustPoseForAlliance(adjustedPose));
        }
        // Verify that for Blue alliance the pose is not changed
        try (MockedStatic<DriverStation> ds = Mockito.mockStatic(DriverStation.class)) {
            ds.when(DriverStation::getAlliance).thenReturn(DriverStation.Alliance.Blue);
            Pose2d startPose = new Pose2d(10, 10, new Rotation2d(-Math.PI / 4));
            assertEquals(startPose, AllianceUtil.adjustPoseForAlliance(startPose));
        }
    }
}