package org.tahomarobotics.robot.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class FudgeableTranslation extends Translation2d {
    private static final Logger logger = LoggerFactory.getLogger(FudgeableTranslation.class);

    private double xRed = 0, xBlue = 0,
                   yRed = 0, yBlue = 0;
    /**
     * Meters and Radians
     */
    public FudgeableTranslation(double x, double y) {
        super(x, y);
    }
    public static FudgeableTranslation newWithInches(double x, double y) {
        return new FudgeableTranslation(Units.inchesToMeters(x), Units.inchesToMeters(y));
    }

    public FudgeableTranslation(Translation2d translation) {
        super(translation.getX(), translation.getY());
    }

    public FudgeableTranslation withXFudge(double blue, double red) {
        xRed = red;
        xBlue = blue;
        return this;
    }

    public FudgeableTranslation withXFudgeInches(double blue, double red) {
        xRed = Units.inchesToMeters(red);
        xBlue = Units.inchesToMeters(blue);
        return this;
    }

    public FudgeableTranslation withYFudge(double blue, double red) {
        yRed = red;
        yBlue = blue;
        return this;
    }

    public FudgeableTranslation withYFudgeInches(double blue, double red) {
        yRed = Units.inchesToMeters(red);
        yBlue = Units.inchesToMeters(blue);
        return this;
    }

    public Translation2d getRedTranslation() {
        return new Translation2d(getX() + xRed, getY() + yRed);
    }

    public Translation2d getBlueTranslation() {
        return new Translation2d(getX() + xBlue, getY() + yBlue);
    }

    public Translation2d getTranslation(DriverStation.Alliance alliance) {
        return
                alliance == DriverStation.Alliance.Blue ?
                        getBlueTranslation() :
                        getRedTranslation();
    }
}
