package org.tahomarobotics.robot.lights;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.OperatorArmMoveSelection;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.SubsystemIF;
import org.tahomarobotics.robot.grabber.Grabber;
import org.tahomarobotics.robot.grabber.GrabberConstants;
import org.tahomarobotics.robot.util.SparkMaxHelper;

public class LED extends SubsystemBase implements SubsystemIF {

    private static final Logger logger = LoggerFactory.getLogger(LED.class);
    private static final LED INSTANCE = new LED();

    public double color = LEDConstants.PARTY_MODE;

    private final Spark Blinkin;

    private LED() {
        Blinkin = new Spark(RobotMap.BLINKIN);
    }

    public static LED getInstance() {
        return INSTANCE;
    }

    public void setLEDColor(double percentage) {
        Blinkin.set(percentage);
    }

    @Override
    public void periodic() {
        setLEDColor(color);
    }
}
