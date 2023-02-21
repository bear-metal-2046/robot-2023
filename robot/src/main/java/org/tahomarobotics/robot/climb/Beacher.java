package org.tahomarobotics.robot.climb;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.SubsystemIF;
import org.tahomarobotics.robot.util.SparkMaxConfig;
import org.tahomarobotics.robot.util.SparkMaxHelper;

public class Beacher extends SubsystemBase implements SubsystemIF {
    private static final Logger logger = LoggerFactory.getLogger(Beacher.class);
    private final CANSparkMax beacherMotor;
    private static final Beacher BEACHER_INSTANCE =
            new Beacher(ClimbConstants.createBeacherConfig(RobotMap.BEACHER, false));
    public static Beacher getBeacherInstance() {
        return BEACHER_INSTANCE;
    }

    public Beacher(SparkMaxConfig config) {
        this.beacherMotor = new CANSparkMax(config.canId, CANSparkMaxLowLevel.MotorType.kBrushed);
        if (SparkMaxHelper.needsConfiguring(logger, config, beacherMotor)) {
            logger.warn("Configuring Beacher Motor");
            SparkMaxHelper.configure(logger, config, beacherMotor);
        }
    }

    @Override
    public Beacher initialize() {
        return this;
    }

    public void runBeacher(double power) {
        beacherMotor.set(power);
    }

    public void stopBeacher() {
        beacherMotor.set(0);
    }
}
