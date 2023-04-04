package org.tahomarobotics.robot.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;
import org.tahomarobotics.robot.util.OrdinaryLeastSquares;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.fail;
import static org.tahomarobotics.robot.arm.ArmConstants.ARM_PHYSICAL_PROPERTIES;
import static org.tahomarobotics.robot.arm.ArmFeedForward.*;

public class ArmOLSTest {

    private final ArmKinematics kinematics = new ArmKinematics();
    private final ArmFeedForward feedForward = new ArmFeedForward();

    private final OrdinaryLeastSquares olsG1 = new OrdinaryLeastSquares(1);
    private final OrdinaryLeastSquares olsG2 = new OrdinaryLeastSquares(1);

    @Test
    void testFeedforwardGravitySolve()  {

        double MIN_ANGLE = -Math.PI/2;
        double MAX_ANGLE = Math.PI/2;
        double INCR_ANGLE = (MAX_ANGLE - MIN_ANGLE)/20;

        double MIN_DIST = 0.1;
        double MAX_DIST =
                ARM_PHYSICAL_PROPERTIES.upperArm().length() +
                        ARM_PHYSICAL_PROPERTIES.foreArm().length();
        double INCR_DIST = (MAX_DIST - MIN_DIST) / 20;

        double time = 0;
        Rotation2d zero = new Rotation2d();

        for (double theta = MIN_ANGLE; theta <= MAX_ANGLE; theta += INCR_ANGLE) {
            for (double range = MIN_DIST; range <= MAX_DIST; range += INCR_DIST) {
                time += 0.001;
                double x = range * Math.cos(theta);
                double y = range * Math.sin(theta);
                Translation2d position = new Translation2d(x, y);

                try {

                    var armState = kinematics.inverseKinematics(0, position);
                    double cos_t1 = Math.cos(armState.shoulder.position());
                    double cos_t1_t2 = Math.cos(armState.shoulder.position() + armState.elbow.position());

                    var voltages = feedForward.calculate(armState, armState);

                    var current1 = MOTOR_SHOULDER.getCurrent(0, voltages.shoulder());
                    var torque1 = MOTOR_SHOULDER.getTorque(current1) / ARM_PHYSICAL_PROPERTIES.upperArm().gearReduction();

                    var current2 = MOTOR_ELBOW.getCurrent(0, voltages.elbow());
                    var torque2 = MOTOR_ELBOW.getTorque(current2) / ARM_PHYSICAL_PROPERTIES.foreArm().gearReduction();

                    System.out.printf("Shoulder: %7.3f, %7.3f, %7.3f, Elbow: %7.3f, %7.3f%n", torque1, cos_t1, cos_t1_t2, torque2, cos_t1_t2);

                    olsG1.add(new double[] {torque1-torque2, cos_t1});
                    olsG2.add(new double[] {torque2, cos_t1_t2});

                } catch (Exception e) {

                }

            }
        }
        double g = 9.80665;

        var g1 = olsG1.calculate();

        var g2 = olsG2.calculate();

        double expected1 = k_m1glc1_m2gl1;
        double expected2 = k_m2glc2;

        System.out.printf("G1: %7.3f, (%7.3f) G2: %7.3f  (%7.3f)%n", g1[0], expected1, g2[0], expected2);

        assertEquals(expected1, g1[0], 0.0001, "G1");
        assertEquals(expected2, g2[0],  0.0001, "G2");
    }
}
