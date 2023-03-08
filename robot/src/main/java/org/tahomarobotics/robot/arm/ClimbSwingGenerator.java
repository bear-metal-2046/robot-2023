package org.tahomarobotics.robot.arm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

import java.util.List;

public class ClimbSwingGenerator {

    public static ArmTrajectory generateTrajectory(double maxVel, double accelTime, double constTime, double decelTime, double elbowAngle, double startAngle, double midAngle, double endAngle) {
        ArmTrajectory trajectory1;
        ArmTrajectory trajectory2;
        TrajectoryConfig cfg1;
        TrajectoryConfig cfg2;

        double decelTimeSq = decelTime * decelTime;
        double accelTimeSq = accelTime * accelTime;
        double accelTimeTimesConstTime = accelTime * constTime;

        double distance1 = midAngle - startAngle;
        double distance2 = endAngle - midAngle;
        double acceleration = distance1/((1.0/2.0) * accelTimeSq + accelTimeTimesConstTime);
        double velocity = accelTime * acceleration;
        double deceleration = (2 * (distance2 - (velocity * decelTime))) / decelTimeSq;

        cfg1 = new TrajectoryConfig(maxVel, acceleration);
        cfg1.setEndVelocity(maxVel);
        cfg2 = new TrajectoryConfig(maxVel, deceleration); // This technically allows for the absolute value of deceleration to be used as well,
        // but it works for this scenario because we don't care about the positive acceleration here
        cfg2.setStartVelocity(maxVel);

        // This generates the two trajectories that we will splice (concatenate) together based on the values we have calculated
        List<Translation2d> midpoints = List.of();
        trajectory1 = new ArmTrajectory(TrajectoryGenerator.generateTrajectory(new Pose2d(new Translation2d(startAngle, elbowAngle), new Rotation2d()), midpoints, new Pose2d(new Translation2d(midAngle, elbowAngle), new Rotation2d()), cfg1));
        trajectory2 = new ArmTrajectory(TrajectoryGenerator.generateTrajectory(new Pose2d(new Translation2d(midAngle, elbowAngle), new Rotation2d()), midpoints, new Pose2d(new Translation2d(endAngle, elbowAngle), new Rotation2d()), cfg2));

        return trajectory1.concatenate(trajectory2);
    }
}
