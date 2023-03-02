package org.tahomarobotics.robot.auto;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.util.ChartData;

import java.util.ArrayList;
import java.util.List;

public class TrajectoryCommand extends SwerveControllerCommand {

    private static final Logger log = LoggerFactory.getLogger(TrajectoryCommand.class);
    private final Trajectory trajectory;

    private final ChartData posXData = new ChartData("Path Motion", "Time (sec)", "Position",
            new String[] {"expected x", "actual x"});
    private final ChartData posYData = new ChartData("Path Motion", "Time (sec)", "Position",
            new String[] {"expected y", "actual y"});
    private final ChartData hdgData = new ChartData("Path Motion", "Time (sec)", "Angle",
            new String[] {"expected hdg", "actual hgd"});
    private final List<Pose2d> actualTrajectory = new ArrayList<>();

    private final Timer timer;

    private final Chassis chassis = Chassis.getInstance();

    private final HolonomicDriveController controller;
    private final Rotation2d heading;

    public TrajectoryCommand(Trajectory trajectory) {
        this(trajectory, getFinalTrajectoryHeading(trajectory), createController());
    }

    public TrajectoryCommand(Trajectory trajectory, Rotation2d heading) {
        this(trajectory, heading, createController());
    }

    private TrajectoryCommand(Trajectory trajectory, Rotation2d heading, HolonomicDriveController controller) {
        super(trajectory,
                Chassis.getInstance()::getPose,
                Chassis.getInstance().getSwerveDriveKinematics(),
                controller,
                () -> heading,
                Chassis.getInstance()::setSwerveStates,
                Chassis.getInstance()
        );
        this.trajectory = trajectory;
        this.controller = controller;
        this.heading = heading;
        timer = new Timer();
    }

    private static Rotation2d getFinalTrajectoryHeading(Trajectory trajectory) {
        var states = trajectory.getStates();
        var finalPose = states.get(states.size()-1).poseMeters;
        return finalPose.getRotation();
    }

    private static HolonomicDriveController createController() {
        PIDController xController = new PIDController(0,0,0);
        PIDController yController = new PIDController(0,0,0);
        ProfiledPIDController headingController = new ProfiledPIDController(2.05, 0, 0, new TrapezoidProfile.Constraints(Math.PI, Math.PI));
        HolonomicDriveController controller = new HolonomicDriveController(xController, yController, headingController);

        // TODO: HolonomicDriveController sets this to 0 to 2PI
        headingController.enableContinuousInput(-Math.PI, Math.PI);

        return controller;
    }

    /**
     * Calculates the velocity and acceleration such that the heading change completes in 90% of the trajectory move.
     */
    private static TrapezoidProfile.Constraints createHeadingChangeConstraints(Trajectory trajectory, Rotation2d heading) {
        Rotation2d currentHeading = Chassis.getInstance().getPose().getRotation();
        double tv = 0.5 * trajectory.getTotalTimeSeconds();
        double ta = 0.2 * trajectory.getTotalTimeSeconds();
        double maxVelocity = (heading.minus(currentHeading)).getRadians()/(tv+ta);
        double maxAcceleration = maxVelocity / ta;
        return new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);
    }

    @Override
    public void initialize() {
        super.initialize();
        timer.reset();
        timer.start();
        log.error("Cleared Chart data");
        posXData.clear();
        posYData.clear();
        hdgData.clear();
        actualTrajectory.clear();
        actualTrajectory.add(chassis.getPose());

        // set the speed of the heading change
        controller.getThetaController().setConstraints(createHeadingChangeConstraints(trajectory, heading));
    }

    @Override
    public void execute() {
        super.execute();
//        log.info("TrajectoryCommand.execute - Pose: " + chassis.getPose());

        double time = timer.get();

        Trajectory.State desiredPose = trajectory.sample(time);

        posXData.addData(new double[]{ time,
                desiredPose.poseMeters.getX(),
                Chassis.getInstance().getPose().getX()
        });
        posYData.addData(new double[]{ time,
                desiredPose.poseMeters.getY(),
                Chassis.getInstance().getPose().getY()
        });
        hdgData.addData(new double[]{ time,
                desiredPose.poseMeters.getRotation().getDegrees(),
                Chassis.getInstance().getPose().getRotation().getDegrees(),
        });


        // update actual path
        actualTrajectory.add(chassis.getPose());

    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        SmartDashboard.putRaw("XPos", posXData.serialize());
        SmartDashboard.putRaw("YPos", posYData.serialize());
        SmartDashboard.putRaw("Hdg", hdgData.serialize());
        chassis.updateActualTrajectory(actualTrajectory);
        chassis.disable();
        timer.stop();
    }
}