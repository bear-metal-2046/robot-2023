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
import edu.wpi.first.math.util.Units;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.fail;

public class ArmKinematicsTest {

    private final static double TOLERANCE_ANGLE = Units.degreesToRadians(1.0);
    private final static double TOLERANCE_DISTANCE = Units.inchesToMeters(0.75);

    private final static double EPSILON = Units.inchesToMeters(0.001);

    private final static double L1 = Units.inchesToMeters(29.575);
    private final static double L2 = Units.inchesToMeters(30.8);
    private final ArmKinematics kinematics = new ArmKinematics(L1, L2);

    private static final Rotation2d ZERO_ANGLE = new Rotation2d();

    private record PositionCondition(String name, Pose2d position, ArmState armState){}

    private PositionCondition createPositionCondition(double x, double y, double shoulder, double elbow, String name) {
        Translation2d position = new Translation2d(Units.inchesToMeters(x), Units.inchesToMeters(y));
        ArmState armState = new ArmState(0, Units.degreesToRadians(shoulder), Units.degreesToRadians(elbow), null);
        return new PositionCondition(name, new Pose2d(position, ZERO_ANGLE), armState);
    }
    private final PositionCondition[] POSITIONS = {
            createPositionCondition( 10.000, -6,  53.88, -157.85, "hopper"),
            createPositionCondition( 30.000, -8,  46.08, -118.14, "floor"),
            createPositionCondition( 30.000, 20,  88.60, -106.69, "place level 1"),
            createPositionCondition( 30.000, 30,  91.54,  -91.73, "place level 2"),
            createPositionCondition( 48.000, 35,  46.59,  -20.57, "way out and up"),
            createPositionCondition(-29.275, 35, 171.84,  -81.84, "forearm vertical"),
            createPositionCondition(-48.000, 35, 154.40,  -20.57, "to the back"),
    };

    @Test
    void testInverseKinematics() {

        double time = 0.0;

        for(PositionCondition c : POSITIONS) {

            try {
                ArmState armState = kinematics.inverseKinematics(time, c.position);

                assertEquals(c.armState.shoulder.position(), armState.shoulder.position(), TOLERANCE_ANGLE,
                        c.name + " shoulder " +
                                Units.radiansToDegrees(c.armState.shoulder.position()) + " <> " +
                                Units.radiansToDegrees(armState.shoulder.position()));

                assertEquals(c.armState.elbow.position(), armState.elbow.position(), TOLERANCE_ANGLE,
                        c.name + " elbow " +
                                Units.radiansToDegrees(c.armState.elbow.position()) + " <> " +
                                Units.radiansToDegrees(armState.elbow.position()));

                time += 0.001;

            } catch (ArmKinematics.KinematicsException e) {
                fail("inverseKinematics threw exception with " + c.name, e);
            }

        }
    }

    @Test
    void testForwardKinematics() {
        for(PositionCondition c : POSITIONS) {
            Translation2d p = kinematics.forwardKinematics(c.armState);

            assertEquals(c.position.getX(), p.getX(), TOLERANCE_DISTANCE,
                    c.name + " X " +
                    Units.metersToInches(c.position.getX()) + " <> " +
                    Units.metersToInches(p.getX()));

            assertEquals(c.position.getY(), p.getY(),  TOLERANCE_DISTANCE,
                    c.name + " Y " +
                     Units.metersToInches(c.position.getY()) + " <> " +
                     Units.metersToInches(p.getY()));

        }
    }
    @Test
    void testAllValidPositions() {

        double MIN_ANGLE = 0;
        double MAX_ANGLE = Math.PI * 2;
        double INCR_ANGLE = (MAX_ANGLE - MIN_ANGLE)/100;

        double MIN_DIST = 0.1;
        double MAX_DIST = L1 + L2;
        double INCR_DIST = (MAX_DIST - MIN_DIST) / 100;

        double time = 0;
        Rotation2d zero = new Rotation2d();

        for (double theta = MIN_ANGLE; theta <= MAX_ANGLE; theta += INCR_ANGLE) {
            for (double range = MIN_DIST; range <= MAX_DIST; range += INCR_DIST) {
                time += 0.001;
                double x = range * Math.cos(theta);
                double y = range * Math.sin(theta);
                Pose2d position = new Pose2d(new Translation2d(x, y), zero);

                try {
                    ArmState state = kinematics.inverseKinematics(time, position);

                    Translation2d result = kinematics.forwardKinematics(state);

                    assertEquals(x, result.getX(), EPSILON);
                    assertEquals(y, result.getY(), EPSILON);

                } catch (ArmKinematics.KinematicsException e) {
                    fail("inverseKinematics threw exception with " + position, e);
                }

            }
        }
    }
}
