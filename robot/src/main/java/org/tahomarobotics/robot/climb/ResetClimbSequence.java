package org.tahomarobotics.robot.climb;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import org.tahomarobotics.robot.arm.Arm;
import org.tahomarobotics.robot.arm.ArmMoveCommand;
import org.tahomarobotics.robot.arm.ArmMovements;
import org.tahomarobotics.robot.wrist.WristMoveCommand;
import org.tahomarobotics.robot.wrist.WristPosition;

public class ResetClimbSequence extends ParallelCommandGroup {
    private final Paw left = Paw.getLeftInstance();
    private final Paw right = Paw.getRightInstance();
    private static final double FINAL_ANGLE = Units.degreesToRadians(20);

    private static final double VELOCITY = 2.5;

    public ResetClimbSequence() {
        addCommands(
                new WristMoveCommand(WristPosition.STOW, 0.5)
                        .andThen(new ProxyCommand(() -> new ArmMoveCommand("Reset Climb",
                                ArmMovements.createPositionToStowTrajectory(
                                        Arm.getInstance().getCurrentPosition(),
                                        new Translation2d(Units.inchesToMeters(11.0), Units.inchesToMeters(0.0)),
                                        new TrajectoryConfig(4, 4)
                                )))),
                new PawCommand(left, FINAL_ANGLE, VELOCITY),
                new PawCommand(right, FINAL_ANGLE, VELOCITY)
        );
    }
}