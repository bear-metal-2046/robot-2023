package org.tahomarobotics.robot.auto;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.util.ChartData;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class TrajectoryCommand extends CommandBase {

    private static final Logger logger = LoggerFactory.getLogger(TrajectoryCommand.class);

    public enum TurnDirection { COUNTER_CLOCKWISE, CLOCKWISE }
    private final Trajectory trajectory;

    private final ChartData velData = new ChartData("Path Motion", "Time (sec)", "Velocity",
            new String[]{"expected vel", "actual vel"});
    private final ChartData hdgData = new ChartData("Path Motion", "Time (sec)", "Angle",
            new String[]{"expected hdg", "actual hgd"});
    private final List<Pose2d> actualTrajectory = new ArrayList<>();

    private final Timer timer = new Timer();

    private final Chassis chassis = Chassis.getInstance();

    private final HolonomicDriveController controller;
    private final Rotation2d heading;

    private final boolean hasEndVelocity;

    private final String name;

    private static class HeadingSupplier implements Supplier<Rotation2d> {

        private final Timer timer = new Timer();

        private Rotation2d startHeading = new Rotation2d();

        private final Rotation2d endHeading;
        private final double turnStart;
        private final double finalStart;

        private final double dir;

        private Rotation2d midAngle;

        public HeadingSupplier(Rotation2d endHeading, double turnStart, double finalStart, TurnDirection turnDirection) {
            this.turnStart = turnStart;
            this.dir = turnDirection == TurnDirection.COUNTER_CLOCKWISE ? 1.0 : -1.0;
            this.endHeading = endHeading;
            this.finalStart=finalStart;
        }
        public void initialize() {
            timer.restart();
            startHeading = Chassis.getInstance().getPose().getRotation();
            double midAngle = (endHeading.getRadians() + startHeading.getRadians())/2;
            if ((midAngle - startHeading.getRadians()) * dir < 0) {
                midAngle += dir * Math.PI;
            }
            this.midAngle = new Rotation2d(midAngle);
        }
        @Override
        public Rotation2d get() {

            Rotation2d heading = startHeading;

            // wait for start timing
            if (timer.hasElapsed(turnStart)) {
                heading = timer.hasElapsed(finalStart) ? endHeading : midAngle;
            }

            // return ending heading
            return heading;
        }
    }

    private final HeadingSupplier headingSupplier;

    private final double turnDuration;

    private static TurnDirection defaultTurn(Rotation2d heading) {
        return heading.getRadians() < 0 ? TurnDirection.CLOCKWISE : TurnDirection.COUNTER_CLOCKWISE;
    }

    public TrajectoryCommand(String name, Trajectory trajectory, Rotation2d heading) {
        this(name, trajectory, heading, 0d, 1d, defaultTurn(heading), false);
    }

    public TrajectoryCommand(String name, Trajectory trajectory, Rotation2d heading, double turnStart, double turnEnd) {
        this(name, trajectory, heading, turnStart, turnEnd, defaultTurn(heading), false);
    }

    public TrajectoryCommand(String name, Trajectory trajectory, Rotation2d heading, double turnStart, double turnEnd, TurnDirection turnDirection) {
        this(name, trajectory, heading, turnStart, turnEnd, turnDirection, false);
    }

    public TrajectoryCommand(String name, Trajectory trajectory, Rotation2d heading, double turnStart, double turnEnd, TurnDirection turnDirection, boolean hasEndVelocity) {
        this(name, trajectory,
                new HeadingSupplier(heading, turnStart * trajectory.getTotalTimeSeconds(), (turnStart+turnEnd)/2*trajectory.getTotalTimeSeconds(), turnDirection),
                heading,
                (turnEnd - turnStart) * trajectory.getTotalTimeSeconds(),
                hasEndVelocity,
                createController());
    }

    private TrajectoryCommand(String name, Trajectory trajectory, HeadingSupplier headingSupplier, Rotation2d heading, double turnDuration, boolean hasEndVelocity, HolonomicDriveController controller) {
        this.name = name;
        this.trajectory = trajectory;
        this.controller = controller;
        this.heading = heading;
        this.hasEndVelocity = hasEndVelocity;
        this.headingSupplier = headingSupplier;
        this.turnDuration = turnDuration;

        addRequirements(Chassis.getInstance());
    }

    private static HolonomicDriveController createController() {
        PIDController xController = new PIDController(5, 0, 0);
        PIDController yController = new PIDController(5, 0, 0);
        ProfiledPIDController headingController = new ProfiledPIDController(5, 0, 0.25, new TrapezoidProfile.Constraints(Math.PI, Math.PI));
        HolonomicDriveController controller = new HolonomicDriveController(xController, yController, headingController);

        // TODO: HolonomicDriveController sets this to 0 to 2PI
        headingController.enableContinuousInput(-Math.PI, Math.PI);

        return controller;
    }

    /**
     * Calculates the velocity and acceleration such that the heading change completes in 90% of the trajectory move.
     */
    private static TrapezoidProfile.Constraints createHeadingChangeConstraints(double turnDuration, Rotation2d heading) {
        Rotation2d currentHeading = Chassis.getInstance().getPose().getRotation();
        double tv = 0.5 * turnDuration;
        double ta = 0.2 * turnDuration;
        double maxVelocity = Math.abs(heading.minus(currentHeading).getRadians()) / (tv + ta);
        double maxAcceleration = maxVelocity / ta;
        return new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);
    }



    @Override
    public void initialize() {
        controller.getThetaController().reset(Chassis.getInstance().getPose().getRotation().getRadians());
        headingSupplier.initialize();
        super.initialize();
        timer.restart();
        velData.clear();
        hdgData.clear();
        actualTrajectory.clear();
        actualTrajectory.add(chassis.getPose());

        // set the speed of the heading change
        controller.getThetaController().setConstraints(createHeadingChangeConstraints(turnDuration, heading));
    }

    @Override
    public void execute() {
        super.execute();
//        log.info("TrajectoryCommand.execute - Pose: " + chassis.getPose());

        double time = timer.get();
        Chassis chassis = Chassis.getInstance();

        Trajectory.State desiredPose = trajectory.sample(time);

        var targetChassisSpeeds =
                controller.calculate(chassis.getPose(), desiredPose, headingSupplier.get());

        chassis.drive(targetChassisSpeeds, false);

        velData.addData(new double[]{time,
                desiredPose.velocityMetersPerSecond,
                Chassis.getInstance().getVelocity()
        });
        hdgData.addData(new double[]{time,
                desiredPose.poseMeters.getRotation().getDegrees(),
                Chassis.getInstance().getPose().getRotation().getDegrees(),
        });


        // update actual path
        actualTrajectory.add(chassis.getPose());

    }

    @Override
    public boolean isFinished() {
        if (!timer.hasElapsed(trajectory.getTotalTimeSeconds())) return false;
        Pose2d lastPose = trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters;

        return (lastPose.getTranslation().getDistance(chassis.getPose().getTranslation()) < Units.inchesToMeters(0.5) &&
                (Math.abs(lastPose.getRotation().getDegrees() - chassis.getPose().getRotation().getDegrees())) < 1) ||
                timer.hasElapsed(trajectory.getTotalTimeSeconds() + 0.5);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        SmartDashboard.putRaw("Vel", velData.serialize());
        SmartDashboard.putRaw("Hdg", hdgData.serialize());
        chassis.updateActualTrajectory(actualTrajectory);
        if (!hasEndVelocity)
            chassis.drive(new ChassisSpeeds());
        timer.stop();
        logger.info(name + "Trajectory has completed with velocity: " + trajectory.sample(trajectory.getTotalTimeSeconds()).velocityMetersPerSecond);
    }


}