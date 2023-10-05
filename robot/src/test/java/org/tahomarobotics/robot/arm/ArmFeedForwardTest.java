package org.tahomarobotics.robot.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import org.junit.jupiter.api.Test;
import org.tahomarobotics.robot.Robot;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

public class ArmFeedForwardTest {

    private static final double MAX_VOLTAGE = 10;

    private static final double MAX_VELOCITY = Units.inchesToMeters(40);
    private static final double MAX_ACCELERATION = MAX_VELOCITY * 2;

    @Test
    void testTrajectory() {

        ArmKinematics kinematics = new ArmKinematics();
        ArmFeedForward feedForward = new ArmFeedForward();

        // test trajectory
        var trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(30),Units.inchesToMeters(-8),new Rotation2d(Math.PI/2)),
                List.of( new Translation2d[] { new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(30)) } ),
                new Pose2d(Units.inchesToMeters(-30), Units.inchesToMeters(-8), new Rotation2d(Math.PI*3/2)),
                new TrajectoryConfig(MAX_VELOCITY, MAX_ACCELERATION));

        try {
            List<ArmState> armStates = new ArrayList<>();
            for (var s : trajectory.getStates()) {
                var a = kinematics.inverseKinematics(s.timeSeconds, s.poseMeters);
                //System.out.println(a);
                armStates.add(a);
            }

            double dT = Robot.defaultPeriodSecs;

            ArmState currentState = armStates.get(0);
            for (double time = dT; time < trajectory.getTotalTimeSeconds(); time += dT) {

                ArmState desiredState = ArmState.sample(time, armStates);

                ArmFeedForward.FeedForwardVoltages voltages = feedForward.calculate(desiredState, currentState);

                assertTrue(inRange(voltages.shoulder(), -MAX_VOLTAGE, MAX_VOLTAGE), "Shoulder Voltage " + voltages.shoulder());
                assertTrue(inRange(voltages.elbow(), -MAX_VOLTAGE, MAX_VOLTAGE), "Elbow Voltage " + voltages.elbow());

                currentState = desiredState;
            }
        } catch(ArmKinematics.KinematicsException e) {
            fail("Kinematics threw exeception: ", e);
        }
    }

    private boolean inRange(double value, double min, double max) {
        return value == MathUtil.clamp(value, min, max);
    }
}
