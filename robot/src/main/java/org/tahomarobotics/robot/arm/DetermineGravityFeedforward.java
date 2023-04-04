package org.tahomarobotics.robot.arm;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.util.OrdinaryLeastSquares;
import org.tahomarobotics.robot.wrist.WristPosition;

import static org.tahomarobotics.robot.arm.ArmConstants.ARM_PHYSICAL_PROPERTIES;
import static org.tahomarobotics.robot.arm.ArmMovements.STOW;
import static org.tahomarobotics.robot.arm.ArmMovements.createPositionToStowTrajectory;

public class DetermineGravityFeedforward extends CommandBase {

    private static final Logger logger = LoggerFactory.getLogger(DetermineGravityFeedforward.class);

    private record TestPoint(Translation2d arm, WristPosition wrist) {}

    private static final double EPSILON = Units.degreesToRadians(0.1);
    private TestPoint testPoint = null;
    private Command executingCommand = null;

    private ArmState armState = new ArmState();

    private int count = 0;

    private final OrdinaryLeastSquares olsG1 = new OrdinaryLeastSquares(1);
    private final OrdinaryLeastSquares olsG2 = new OrdinaryLeastSquares(1);

    private final ArmKinematics kinematics = new ArmKinematics();

    private int index = 0;
    public DetermineGravityFeedforward() {
    }

    @Override
    public void initialize() {
         index = 0;
    }

    @Override
    public void execute() {
        if (executingCommand == null) {

            testPoint = stepTestPoint(testPoint);

            executingCommand = new ArmMovements.ArmMove("testPoint["+index+"]",
                            createPositionToStowTrajectory(Arm.getInstance().getCurrentPosition(), testPoint.arm),
                            testPoint.wrist).createArmWristMoveCommand();
            executingCommand.schedule();
            count = 3;


        } else if (!executingCommand.isScheduled()) {

            ArmState armState = Arm.getInstance().getCurrentArmState();

            if (isMovingStopped(this.armState, armState)) {
                if (count-- <= 0) {
                    executingCommand = null;


                    var elec = Arm.getInstance().getArmElectricalInfo();

                    double i1 = ArmFeedForward.MOTOR_SHOULDER.getCurrent(0, elec.shoulderVoltage());
                    double t1 = ArmFeedForward.MOTOR_SHOULDER.getTorque(i1) / ARM_PHYSICAL_PROPERTIES.upperArm().gearReduction();
                    double i2 = ArmFeedForward.MOTOR_ELBOW.getCurrent(0, elec.elbowVoltage());
                    double t2 = ArmFeedForward.MOTOR_ELBOW.getTorque(i2)  / ARM_PHYSICAL_PROPERTIES.foreArm().gearReduction();
                    double cos_t1 = Math.cos(armState.shoulder.position());
                    double cos_t1_t2 = Math.cos(armState.shoulder.position() + armState.elbow.position());
                    olsG1.add(new double[] { t1 - t2, cos_t1});
                    olsG2.add(new double[] { t2, cos_t1_t2});

                    index++;
                }
            } else {
                count = 3;
            }
            this.armState = armState;
        }
    }

    private TestPoint stepTestPoint(TestPoint prev) {

        Translation2d position;

        if (prev == null) {
            position = new Translation2d(Units.inchesToMeters(10), Units.inchesToMeters(-5));
        } else {        // x range = 10 to 55, y range = -10 to 30 every 5 inches
            if (prev.arm.getY() >= Units.inchesToMeters(30) && prev.arm.getX() >= Units.inchesToMeters(55)) {
                return new TestPoint(STOW, WristPosition.STOW);
            } else if (prev.arm.getY() >= Units.inchesToMeters(30)) {
                // reset to minimum y and increment x
                position = new Translation2d(prev.arm.getX() + Units.inchesToMeters(5), Units.inchesToMeters(-5));
            } else {
                // increment y
                position = new Translation2d(prev.arm.getX(), prev.arm.getY() + Units.inchesToMeters(5));
            }
        }

        var testPoint = new TestPoint(position, position.getY() > Units.inchesToMeters(25) ? WristPosition.HIGH_POLE_PLACE : WristPosition.CUBE_COLLECT);


        if (kinematics.validateArmPosition(testPoint.arm)) {
            return testPoint;
        }

        return stepTestPoint(testPoint);
    }

    private boolean isMovingStopped(ArmState prev, ArmState current) {
        return
                Math.abs(current.shoulder.position() - prev.shoulder.position()) < EPSILON &&
                Math.abs(current.elbow.position() - prev.elbow.position()) < EPSILON;
    }

    @Override
    public void end(boolean interrupted) {

        // process data
        double g1 = olsG1.calculate()[0];
        double g2 = olsG2.calculate()[0];

        // m1glc1_m2gl1 = 50.499177
        // m2glc2 = 30.598695

        logger.info("Completed determining Gravity Feedforward g1=" + g1 + " g2=" + g2);
    }

    @Override
    public boolean isFinished() {
       return testPoint.arm == STOW && executingCommand == null;
    }
}
