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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class ArmKinematics {

    private static final Logger logger = LoggerFactory.getLogger(ArmKinematics.class);

    public static class KinematicsException extends Exception {
        public KinematicsException(String message) {
            super(message);
        }
    }

    private ArmState armState = null;

    private final double l1;
    private final double l2;
    private final double l1SqPlusL2Sq;
    private final double l1SqMinusL2Sq;
    private final double twoL1L2;

    public ArmKinematics() {
        this(ArmConstants.ARM_PHYSICAL_PROPERTIES.upperArm().length(), ArmConstants.ARM_PHYSICAL_PROPERTIES.foreArm().length());
    }

    /**
     * Used for unit testing
     */
    public ArmKinematics(double l1, double l2) {
        this.l1 = l1;
        this.l2 = l2;
        double l1Sq = l1 * l1;
        double l2Sq = l2 * l2;
        l1SqPlusL2Sq = l1Sq + l2Sq;
        l1SqMinusL2Sq = l1Sq - l2Sq;
        twoL1L2 = 2d * l1 * l2;
    }

    public ArmState inverseKinematics(double time, Pose2d position) throws KinematicsException {
        return inverseKinematics(time, position.getTranslation());
    }

    /**
     * Calculates the joint positions, velocities and accelerations from the provided
     * cartesian coordinates.  The previously calculated arm state is retained for calculating the
     * velocities and accelerations.  This is done in ArmState.  The new time is required for
     * those derivatives.
     *
     * @param time - time in decimal seconds
     * @param position - pose2d usually provided by a trajectory sample.
     * @return ArmState containing arm joint angles
     */
    public ArmState inverseKinematics(double time, Translation2d position) throws KinematicsException {

        double x = position.getX();
        double y = position.getY();
        if (Double.isNaN(x) || Double.isNaN(y)) {
            throw new KinematicsException("Argument is NaN: " + position);
        }

        /*
         * Length l3 is distance from the should joint to the end-effector.
         * This is calculated by using the hypothesis.
         */
        double l3Sq = x * x + y * y;
        double l3 = Math.sqrt(l3Sq); // never NaN

        /*
         * Angle phi is the angle between upper-arm and fore-arm.  This uses the
         * law of cosine to determine the 3 known sides. Force to 0 to PI.
         */
        double cosPhi = MathUtil.clamp((l1SqPlusL2Sq - l3Sq) / twoL1L2, -1.0, 1.0);
        double phi = Math.acos(cosPhi);

        /*
         * Angle beta is the angle between the x-axis and the
         * end-effector.
         */
        double beta = Math.atan2(y, x); // never NaN

        /*
         * Angle alpha is the angle between the l3 (line from shoulder to end-effector)
         * and the upper arm.  Using the law of sines arcsin[ L2 / L3 * sin(phi)] has
         * too much error when using inverse follow by forward kinematics. Using the
         * law of cosine only using arm distances and the hypothesis and provides a very
         * low tolerance.
         */
        double cosAlpha = MathUtil.clamp(( l1SqMinusL2Sq + l3Sq ) / ( 2d * l1 * l3 ), -1.0, 1.0);
        double alpha = Math.acos( cosAlpha );

        /*
         * Shoulder joint or theta1 is calculated by simply adding beta and alpha getting
         * the angle from the x-axis to the upper-arm.
         */
        double shoulderPosition = MathUtil.angleModulus(beta + alpha);

        /*
         * Elbow joint or theta2 is calculated by subtracting 180 degrees from phi since this angle is
         * defined the upper-arm to the fore-arm with postive angle counter-clockwise looking from the right
         * side of the robot.  The encoder will be configured to match.
         */
        double elbowPosition = MathUtil.angleModulus(phi - Math.PI);

        /*
         * ArmState holds the joint positions just calculated along with velocities and accelerations calculated
         * from the previous arm state and the current time.
         */
        armState = new ArmState(time, shoulderPosition, elbowPosition, armState);

        if (Double.isNaN(time) || Double.isNaN(shoulderPosition) || Double.isNaN(elbowPosition)) {
            throw new KinematicsException("Something went wrong: " + armState);
        }

        if (logger.isDebugEnabled()) {
            logger.debug(String.format("Inverse X:%7.3f Y:%7.3f phi:%7.3f beta:%7.3f alpha:%7.3f theta1:%7.3f theta2:%7.3f",
                    Units.metersToInches(x), Units.metersToInches(y),
                    Units.radiansToDegrees(phi), Units.radiansToDegrees(beta), Units.radiansToDegrees(alpha),
                    Units.radiansToDegrees(shoulderPosition), Units.radiansToDegrees(elbowPosition)));
        }

        return armState;
    }

    /**
     * Returns the position of the end-effector with the provided arm joint angles.
     *
     * @param armState - joint angles
     * @return Pose2d of end-effector
     */
    public Translation2d forwardKinematics(ArmState armState) {
        double q1 = armState.shoulder.position();
        double q2 = armState.elbow.position();

        double x = l1 * Math.cos(q1) + l2 * Math.cos(q1 + q2);
        double y = l1 * Math.sin(q1) + l2 * Math.sin(q1 + q2);

        return new Translation2d(x, y);
    }

    boolean inRange(double value , ArmConstants.Range range) {
        return inRange(value, range.min(), range.max());
    }

    boolean inRange(double param, double min, double max) {
        return (param >= min && param <= max);
    }

    public boolean validateTrajectory(Trajectory trajectory) {

        /*
         *
         * Validations:
         *   Vertical Range
         *   Horizontal Range
         *   Shoulder Range
         *   Elbow Range
         *   Robot Exclusion (hint: box with vertical and horizontal ranges)
         */

        for(Trajectory.State a : trajectory.getStates()) {
            if ( !validateArmPosition(a.poseMeters.getTranslation())) return false;
        }
        return true;
    }

    public boolean validateArmPosition(Translation2d p) {
        if ( ! inRange(p.getY(), ArmConstants.VERTICAL_LIMITS) ) {
            logger.error("Trajectory failed validation: y value out of range");
            return false;
        }

        if ( ! inRange(p.getX(), ArmConstants.HORIZONTAL_LIMITS) ) {
            logger.error("Trajectory failed validation: x value out of range");
            return false;
        }

        if ( inRange(p.getY(), ArmConstants.TOWER_EXCLUSION_VERTICAL) && inRange(p.getX(), ArmConstants.TOWER_EXCLUSION_HORIZONTAl)) {
            logger.error("Trajectory failed validation: within tower exclusion");
            return false;
        }

        try {
            ArmState armState = inverseKinematics(0, p);

            if (! inRange(armState.shoulder.position(), ArmConstants.SHOULDER_LIMITS)) {
                logger.error("Trajectory failed validation: shoulder angle out of range");
                return false;
            }
            if (! inRange(armState.elbow.position(), ArmConstants.ELBOW_LIMITS)) {
                logger.error("Trajectory failed validation: elbow angle out of range");
                return false;
            }

        } catch (KinematicsException e) {
            logger.error("Position failed validation", e);
            return false;
        }
        return true;
    }

}


