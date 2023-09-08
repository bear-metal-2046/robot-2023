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

import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class ArmState {

    public static record JointState(double position, double velocity, double acceleration) {
        private JointState(double dT, double position, double velocity, JointState prev) {
            this(position, velocity, (velocity - prev.velocity)/dT);
        }

        public JointState(double dT, double position, JointState prev) {
            // when taking the differences of angles, modulus will keep the results between -pi and +pi
            this(dT, position, MathUtil.angleModulus(position - prev.position)/dT, prev);
        }

        public JointState interpolate(JointState next, double dt, double sampleDelta) {
            double p, v, a, j;

            j = (next.acceleration - acceleration) / sampleDelta;
            a = acceleration + j * dt;
            v = velocity + dt * a;
            p = MathUtil.angleModulus(position + dt * v);
            return new JointState(p, v, a);
        }

        @Override
        public String toString() {
            return String.format("%7.3f, %7.3f, %7.3f", position, velocity, acceleration);
        }
    }

    public final double time;
    public final JointState shoulder;
    public final JointState elbow;

    public ArmState() {
        this(0d, new JointState(0d,0d,0d), new JointState(0d,0d,0d));
    }

    public ArmState(double time, JointState shoulder, JointState elbow) {
        this.time = time;
        this.shoulder = shoulder;
        this.elbow = elbow;
    }


    public ArmState(double time, double shoulderPosition, double elbowPosition, ArmState previousState) {
        this.time = time;

        if (previousState != null) {
            double dT = time - previousState.time;
            if (dT > 0) {
                this.shoulder = new ArmState.JointState(dT, shoulderPosition, previousState.shoulder);
                this.elbow = new ArmState.JointState(dT, elbowPosition, previousState.elbow);
                return;
            }
        }
        this.shoulder = new ArmState.JointState(shoulderPosition, 0, 0);
        this.elbow = new ArmState.JointState(elbowPosition, 0,0);
    }

    public static ArmState sample(double time, List<ArmState> orderedArmStates) {

        ArmState last = orderedArmStates.get(orderedArmStates.size()-1);
        ArmState key = new ArmState(time, null, null);
        if (time >= last.time) {
            return last;
        }

        int index = Collections.binarySearch(orderedArmStates, key,
                Comparator.comparingDouble(armState -> armState.time));
        if (index < 0) {
            index = - (index + 2);
        }
        index = MathUtil.clamp(index, 0, orderedArmStates.size()-2);
        ArmState low = orderedArmStates.get(index);
        ArmState high = orderedArmStates.get(index + 1);

        double sampleDelta = high.time - low.time;
        double dt = time - low.time;
        JointState shoulder = low.shoulder.interpolate(high.shoulder, dt, sampleDelta);
        JointState elbow = low.elbow.interpolate(high.elbow, dt, sampleDelta);

        return new ArmState(time, shoulder, elbow);
    }

    @Override
    public String toString() {
        return String.format("%7.3f, %s, %s", time, shoulder, elbow);
    }
}
