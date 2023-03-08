package org.tahomarobotics.robot.arm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;
import static org.junit.jupiter.api.Assertions.fail;

public class ArmTrajectoryTest {


    @Test
    void testArmTrajectories() {


        ArmMovements.ArmMove startToStow = ArmMovements.START_TO_STOW;
        assertTrue(startToStow.trajectory().isValid());

        for (var trajectory : ArmTrajectory.allTrajectories) {
            assertTrue(trajectory.isValid());
        }

    }

    @Test
    void testPositionToStow() {

        double MIN_ANGLE = 0;
        double MAX_ANGLE = Math.PI * 2;
        double INCR_ANGLE = (MAX_ANGLE - MIN_ANGLE)/100;

        double MIN_DIST = 0.1;
        double MAX_DIST =
                ArmConstants.ARM_PHYSICAL_PROPERTIES.upperArm().length() +
                ArmConstants.ARM_PHYSICAL_PROPERTIES.foreArm().length();
        double INCR_DIST = (MAX_DIST - MIN_DIST) / 100;

        double time = 0;
        Rotation2d zero = new Rotation2d();

        for (double theta = MIN_ANGLE; theta <= MAX_ANGLE; theta += INCR_ANGLE) {
            for (double range = MIN_DIST; range <= MAX_DIST; range += INCR_DIST) {
                time += 0.001;
                double x = range * Math.cos(theta);
                double y = range * Math.sin(theta);
                Translation2d position = new Translation2d(x, y);

                var trajectory = ArmMovements.createPositionToStowTrajectory(position, ArmMovements.STOW);

                if (!trajectory.isValid()) {
                    fail("Invalid Trajectory position:" + position + " STOW:" + ArmMovements.STOW + " Diff:" + (position.minus(ArmMovements.STOW)));
                }


            }
        }
    }
}
