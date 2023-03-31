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

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import static org.tahomarobotics.robot.arm.ArmConstants.ARM_PHYSICAL_PROPERTIES;

public class ArmFeedForward {

    private static double lbsInchSqTokgsMeterSq(double value) { return Units.inchesToMeters(Units.inchesToMeters(Units.lbsToKilograms(value))); }
    private static final Logger logger = LoggerFactory.getLogger(ArmFeedForward.class);

    // upper-arm link (mass, moment of inertia, CoM, length)
    private static final double m1 = ARM_PHYSICAL_PROPERTIES.upperArm().mass();
    private static final double I1 = ARM_PHYSICAL_PROPERTIES.upperArm().inertiaCenterMass();
    private static final double l1 = ARM_PHYSICAL_PROPERTIES.upperArm().length();
    private static final double lc1 = ARM_PHYSICAL_PROPERTIES.upperArm().locationCenterMass();
    private static final double l1Sq = l1 * l1;
    private static final double lc1Sq = lc1 * lc1;


    // forearm link (mass, moment of inertia, CoM)
    // added 6 inch cube to end with (5 lbs)
    private static final double m2 = ARM_PHYSICAL_PROPERTIES.foreArm().mass();
    private static final double I2 = ARM_PHYSICAL_PROPERTIES.foreArm().inertiaCenterMass();
    private static final double lc2 = ARM_PHYSICAL_PROPERTIES.foreArm().locationCenterMass();
    private static final double lc2Sq = lc2 * lc2;

    // gravity acceleration
    private static final double g =  9.80665;
    private static final double k_m1glc1_m2gl1 = 18.39845800340516;
    private static final double k_m2glc2 = 20.375856190104;
    private static final double k_m2lc2 = k_m2glc2 / g;


    static final DCMotor MOTOR_SHOULDER = DCMotor.getNEO(2);
    static final DCMotor MOTOR_ELBOW = DCMotor.getNEO(1);


    public record FeedForwardVoltages(double shoulder, double elbow) {}

    public FeedForwardVoltages calculate(ArmState desiredArmState, ArmState currentArmState) {

        double t1 = currentArmState.shoulder.position();
        double t2 = currentArmState.elbow.position();
        double cos_t1 = Math.cos(t1);
        double cos_t2 = Math.cos(t2);
        double cos_t1_t2 = Math.cos(t1 + t2);

        // update inertial terms
        // -------------------------------------------------------------
        //   H11 - moment of inertia (both links about shoulder joint)
        //         use parallel axis theorem on mass 1 and 2 to shoulder axis
        //         use law of cosines to adjust lc2 distance to shoulder axis
        //         based on the elbow angle (t2)
        double lc2SqPrime = l1Sq + lc2Sq + 2 * l1 + lc2 * cos_t2;
        double H11 = m1 * lc1Sq + I1    // upper-arm contribution
                + m2 * lc2SqPrime + I2; // forearm contribution

        // ------------------------------------------------------------
        //   H22 - moment of inertia (forearm link about elbow joint)
        //         use parallel axis theorem on mass 2 to elbow joint
        double H22 = m2 * lc2Sq + I2;

        // ------------------------------------------------------------
        //   H12 - moment of inertia about the shoulder joint resulting
        //         from coupling torque while accelerating forearm link
        double H12 = m2 * ( lc2Sq + l1 * lc2 * cos_t2 ) + I2;

        // ------------------------------------------------------------
        // calculate torque required for desired accelerations
        double a1 = desiredArmState.shoulder.acceleration();
        double a2 = desiredArmState.elbow.acceleration();
        double torqueInertial1 = H11 * a1 + H12 * a2;
        double torqueInertial2 = H12 * a1 + H22 * a2;

        // update centrifugal terms
        // ------------------------------------------------------------
        //   h  - moment associated with centripetal force of joint velocities.
        //        With shoulder joint velocity acting on forearm mass causes
        //        a torque on the second joint when the elbow joint is at
        //        an angle.
        double sin_t2 = Math.sin(t2);
        double h = k_m2lc2 * sin_t2;
        double w1 = desiredArmState.shoulder.velocity();
        double w2 = desiredArmState.elbow.velocity();
        double torqueCentrifugal1 = - h * w2 * w2;
        double torqueCentrifugal2 =   h * w1 * w1;

        // update coriolis term
        double torqueCoriolis1 = -2 * h  * w1 * w2;

        // update gravity torques
        // ------------------------------------------------------------
        //   G2 - moment associated with the acceleration of gravity
        //        the forearm link mass about the elbow joint.
        double G2 = k_m2glc2 * cos_t1_t2; //  m2 * g * lc2 * cos_t1_t2;

        // ------------------------------------------------------------
        //   G1 - moment associated with the acceleration of gravity
        //        both link masses about the shoulder joint.  Upper-arm
        //        link mass times horizontal distance from shoulder joint.
        //        Forearm link mass times horizontal distance to the same joint.
        double G1 = G2 + k_m1glc1_m2gl1 * cos_t1; // m1 * g * lc1 * cos_t1 + m2 * g * (l1 * cos_t1 + lc2 * cos_t1_t2);




        // calculate shoulder motor voltage
        double torque1 = torqueInertial1 + torqueCentrifugal1 + torqueCoriolis1 + G1;
        double motorTorque1 = torque1 * ARM_PHYSICAL_PROPERTIES.upperArm().gearReduction();
        double motorVelocity1 = w1 / ARM_PHYSICAL_PROPERTIES.upperArm().gearReduction();
        double voltageShoulder = MOTOR_SHOULDER.getVoltage(motorTorque1, motorVelocity1);

        // calculate elbow motor voltage
        double torque2 = torqueInertial2 + torqueCentrifugal2 + G2;
        double motorTorque2 = torque2 * ARM_PHYSICAL_PROPERTIES.foreArm().gearReduction();
        double motorVelocity2 = w2 / ARM_PHYSICAL_PROPERTIES.foreArm().gearReduction();
        double voltageElbow = MOTOR_ELBOW.getVoltage(motorTorque2, motorVelocity2);


        if (logger.isDebugEnabled()) {

            logger.debug(String.format("Shoulder: %7.3f V, %7.3f nM, %7.3f nM, %7.3f nM, %7.3f nM, Elbow: %7.3f V, %7.3f nM, %7.3f nM, %7.3f nM %n",
                    voltageShoulder, torqueInertial1, torqueCentrifugal1, torqueCoriolis1, G1,
                    voltageElbow, torqueInertial2, torqueCentrifugal2, G2));
        }

        return new FeedForwardVoltages(voltageShoulder, voltageElbow);
    }

}
