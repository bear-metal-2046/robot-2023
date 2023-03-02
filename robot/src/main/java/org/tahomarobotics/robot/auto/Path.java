package org.tahomarobotics.robot.auto;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.chassis.Chassis;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.function.Supplier;

public class Path {
    private static final Logger logger = LoggerFactory.getLogger(Path.class);

    interface Transformation {
        Pose2d transform(Pose2d currentPose);
    }

    private final List<Transformation> segmentBuffer;
    private final List<Pair<Boolean, List<Transformation>>> segmentsBuffer;
    private final List<Command> segments;
    private final String name;
    private final TrajectoryConfig config;
    private final List<Trajectory> trajectories;
    // TODO Split by segment
    private final List<String> descriptions;

    private Pose2d startPose;

    public Path(String name, TrajectoryConfig config) {
        this.name = name;
        this.config = config;
        this.segmentBuffer = new ArrayList<>();
        this.segmentsBuffer = new ArrayList<>();
        this.segments = new ArrayList<>();
        this.trajectories = new ArrayList<>();
        this.descriptions = new ArrayList<>();
    }

    /**
     * Ends the current segment then appends the command.
     * @param command Command to run
     */
    public Path command(Supplier<Command> command) {
        segmentEnd();
        segmentBuffer.add(currentPose -> {
            segments.add(command.get());
            return currentPose;
        });
        descriptions.add("Running command " + command.toString());
        return this;
    }

    /**
     * Ends the current segment then appends the command.
     * @param command Command to run
     * @param addDesc Whether to add a description
     */
    public Path command(Supplier<Command> command, boolean addDesc) {
        segmentEnd();
        segmentBuffer.add(currentPose -> {
            segments.add(command.get());
            return currentPose;
        });
        if (addDesc)
            descriptions.add("Running command " + command.toString());
        return this;
    }

    /**
     * Uses the segments of another path.
     * Operates continuously; meaning that you may append on an open path that has not yet closed
     * and continue the current path of this one.
     */
    public Path use(Path path) {
        segmentBuffer.addAll(path.segmentBuffer);
        // TODO Pull descriptions over.
        segmentEnd();
        segments.addAll(path.segments);
        return this;
    }

    /**
     * Waits the specified amount of time in seconds.
     */
    public Path waitCommand(double seconds) {
        command(() -> new WaitCommand(seconds), false);
        descriptions.add("Waiting " + seconds + " seconds.");
        return this;
    }

    /**
     * Drives forward the specified {@code distance}.
     * @param distance Distance in meters to drive forward
     */
    public Path driveForward(double distance) {
        segmentBuffer.add(
            currentPose -> {
                Pose2d nextPose = currentPose
                        .plus(new Transform2d(new Translation2d(distance, 0), new Rotation2d()));
                descriptions.add(String.format(
                        "Driving forward %.2f meters [%s] -> [%s]", distance, currentPose, nextPose
                ));
                return nextPose;
            }
        );
        return this;
    }

    /**
     * Drives backward the specified {@code distance}.
     * @param distance Distance in meters to drive backward
     */
    public Path driveBackwards(double distance) {
        segmentEnd();
        segmentBuffer.add(
                currentPose -> {
                    Pose2d nextPose = currentPose
                            .plus(new Transform2d(new Translation2d(-distance, 0), new Rotation2d()));
                    descriptions.add(String.format(
                            "Driving backwards %.2f meters [%s] -> [%s]", distance, currentPose, nextPose
                    ));
                    return nextPose;
                }
        );
        segmentEndReversed();
        return this;
    }

    /**
     * Move the robot to the {@code targetPose}
     * @param targetPose Absolute (relative to (0,0)) target pose
     */
    public Path driveTo(Pose2d targetPose) {
        segmentBuffer.add(
                currentPose -> {
                    descriptions.add(String.format(
                            "Driving to %s [%s] -> [%s]", targetPose, currentPose, targetPose
                    ));
                    return targetPose;
                }
        );
        return this;
    }

    /**
     * Move the robot to (x, y) with the current rotation.
     * @param x x coordinate in meters
     * @param y y coordinate in meters
     */
    public Path driveTo(double x, double y) {
        segmentBuffer.add(
                currentPose -> {
                    Pose2d newPose = new Pose2d(x, y, currentPose.getRotation());
                    descriptions.add(String.format(
                            "Driving to %s [%s] -> [%s]", newPose, currentPose, newPose
                    ));
                    return newPose;
                }
        );
        return this;
    }

    public Path driveToStart() {
        segmentBuffer.add(
                currentPose -> {
                    descriptions.add(String.format(
                            "Driving to %s [%s] -> [%s]", this.startPose, currentPose, this.startPose
                    ));
                    return this.startPose;
                }
        );
        return this;
    }

    /**
     * Mutates the prior transformation with {@code transformation}
     */
    public Path and(Transformation transformation) {
        int i = segmentBuffer.size()-1;
        Transformation mut = segmentBuffer.get(i);
        segmentBuffer.set(i,
                currentPose -> {
                    descriptions.add("and modifiers <NYI>");
                    return transformation.transform(mut.transform(currentPose));
                }
        );
        return this;
    }

    /**
     * Relative Rotational Modifier
     */
    public Path withRotateBy(double angle) {
        int idx = segmentBuffer.size() - 1;
        Transformation t = segmentBuffer.get(idx);
        segmentBuffer.set(idx,
            currentPose -> {
                Pose2d prev = t.transform(currentPose);
                Rotation2d rot = prev.getRotation().rotateBy(new Rotation2d(angle));
                descriptions.set(idx,
                    String.format(
                            descriptions.get(idx) + " and rotating by %.2f radians [%s] -> [%s]", angle,
                            prev.getRotation().toString(),
                            rot.toString()
                    )
                );
                return new Pose2d(prev.getX(), prev.getY(), rot);
            }
        );
        return this;
    }


    /**
     * Absolute Rotational Modifier
     */
    public Path withRotateTo(double angle) {
        int idx = segmentBuffer.size() - 1;
        Transformation t = segmentBuffer.get(idx);
        segmentBuffer.set(idx,
                currentPose -> {
                    Pose2d prev = t.transform(currentPose);
                    Rotation2d rot = new Rotation2d(angle);
                    descriptions.set(idx,
                            String.format(
                                    descriptions.get(idx) + " and rotating to %.2f radians [%s] -> [%s]", angle,
                                    prev.getRotation().toString(),
                                    rot.toString()
                            )
                    );
                    return new Pose2d(prev.getX(), prev.getY(), rot);
                }
        );
        return this;
    }

    /**
     * Accumulates prior transformations into continuous trajectory;
     * Implicitly called by {@link #build(Pose2d)}
     */
    public Path segmentEnd() {
        segmentEnd(false);
        return this;
    }

    /**
     * Accumulates prior transformations into continuous trajectory in reverse;
     * Implicitly called by {@link #build(Pose2d)}
     */
    public Path segmentEndReversed() {
        segmentEnd(true);
        return this;
    }

    private Path segmentEnd(boolean reversed) {
        if (segmentBuffer.isEmpty())
            return this;

        segmentsBuffer.add(Pair.of(reversed, new ArrayList<>(segmentBuffer)));
        segmentBuffer.clear();
        return this;
    }

    public AutoCommand build(Pose2d startPose) {
        return build(startPose, true);
    }
    public AutoCommand build(Pose2d startPose, boolean startAtCurrentPos) {
        segmentEnd();

        Chassis chassis = Chassis.getInstance();

        this.startPose = startPose;

        Pose2d _startPose = startPose;

        for (var segment : segmentsBuffer) {
            List<Translation2d> translations = new ArrayList<>();
            Iterator<Transformation> iter = segment.getSecond().iterator();
            Pose2d pose = iter.next().transform(_startPose);

            while (iter.hasNext()) {
                Transformation next = iter.next();
                pose = next.transform(pose);
                if (iter.hasNext())
                    translations.add(pose.getTranslation());
            }

            if (pose == _startPose) {
                continue;
            }

            config.setReversed(segment.getFirst());

            Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                    _startPose,
                    translations,
                    pose,
                    config
            );

            _startPose = pose;


            trajectories.add(trajectory);
            // TODO: adding in arbitrary heading changes
            segments.add(new TrajectoryCommand(trajectory));
        }

        AutoCommand auto = new AutoCommand() {
            @Override
            public String getName() {
                return name;
            }

            @Override
            public void onSelection() {
                if (!startAtCurrentPos)
                    chassis.resetOdometry(trajectories.size() > 0 ? trajectories.get(0).getInitialPose() : startPose);
                chassis.updateTrajectory(trajectories.size() > 0 ? trajectories : null);
                logger.info(getName() + " selected.");
            }

            @Override
            public String toString() {
                StringBuilder info = new StringBuilder("\n" + name + " [AutoCommand]\n");
                for (String desc : descriptions)
                    info.append("\t").append(desc).append("\n");
                return info.toString();
            }
        };

        for (Command c : segments) {
            auto.addCommands(c);
        }

        segments.clear();

        return auto;
    }

    public String getName() {
        return name;
    }
}