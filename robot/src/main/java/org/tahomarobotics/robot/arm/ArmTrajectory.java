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
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.List;

public class ArmTrajectory {

    private static final Logger logger = LoggerFactory.getLogger(ArmTrajectory.class);

    private final ArmKinematics kinematics = new ArmKinematics();

    private final Trajectory trajectory;

    private final boolean valid;

    public ArmTrajectory(Translation2d start, Translation2d end, TrajectoryConfig config) {
        this(start, end, end.minus(start).getAngle(), config);
    }

    private ArmTrajectory(Translation2d start, Translation2d end, Rotation2d angle, TrajectoryConfig config) {
        this(new Pose2d(start, angle), List.of(), new Pose2d(end, angle), config);
    }
    public ArmTrajectory(Pose2d start, List<Translation2d> interiorWaypoints, Pose2d end, TrajectoryConfig config)  {
        trajectory = generateTrajectory(start, interiorWaypoints, end, config);
        valid = trajectory != null;
    }

    public boolean isValid() {
        return valid;
    }

    private Trajectory generateTrajectory(Pose2d start, List<Translation2d> interiorWaypoints, Pose2d end, TrajectoryConfig config) {
        Trajectory trajectory = null;
        try {
            double directionSample = end.getTranslation().getDistance(start.getTranslation()) * 0.001;

            Pose2d convertedStart = convertToAngles(start, directionSample);

            List<Translation2d> convertedWaypoints = new ArrayList<>();
            for (var wp : interiorWaypoints) {
                convertedWaypoints.add(convertToAngles(wp));
            }

            Pose2d convertedEnd = convertToAngles(end, directionSample);

            trajectory = TrajectoryGenerator.generateTrajectory(convertedStart, convertedWaypoints, convertedEnd, config);

        } catch (ArmKinematics.KinematicsException|RuntimeException e) {
            logger.error("Failed to create trajectory", e);
        }
        return trajectory;
    }

    private Translation2d convertToAngles(Translation2d position) throws ArmKinematics.KinematicsException {
        ArmState armState = kinematics.inverseKinematics(0, position);
        return new Translation2d(armState.shoulder.position(), armState.elbow.position());
    }

    private Pose2d convertToAngles(Pose2d position, double directionDistance) throws ArmKinematics.KinematicsException {

        Translation2d convertedPosition = convertToAngles(position.getTranslation());

        Translation2d nextPosition = position.getTranslation().plus(new Translation2d(directionDistance, position.getRotation()));

        Translation2d directionPosition = convertToAngles(nextPosition);

        Rotation2d convertedRotation = directionPosition.minus(convertedPosition).getAngle();

        return new Pose2d(convertedPosition, convertedRotation);
    }

    public ArmState sample(double time) {
        var sample = trajectory.sample(time);
        var angle = sample.poseMeters.getRotation();

        return new ArmState(sample.timeSeconds,
                new ArmState.JointState(
                        sample.poseMeters.getX(),
                        sample.velocityMetersPerSecond * angle.getCos(),
                        sample.accelerationMetersPerSecondSq * angle.getCos()
                ),
                new ArmState.JointState(
                        sample.poseMeters.getY(),
                        sample.velocityMetersPerSecond * angle.getSin(),
                        sample.accelerationMetersPerSecondSq * angle.getSin()
                )
        );

    }

    public double getTotalTimeSeconds() {
        return trajectory.getTotalTimeSeconds();
    }

    public ArmState getFinalState() {
        var states = trajectory.getStates();
        var sample = states.get(states.size()-1);
        return new ArmState(sample.timeSeconds,
                new ArmState.JointState(sample.poseMeters.getX(), 0, 0),
                new ArmState.JointState(sample.poseMeters.getY(), 0, 0));
    }
}
