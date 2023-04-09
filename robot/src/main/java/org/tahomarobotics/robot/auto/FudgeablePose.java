package org.tahomarobotics.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class FudgeablePose extends Pose2d {
    private static final Logger logger = LoggerFactory.getLogger(FudgeablePose.class);

    private FudgeableTranslation fTrans;
    private double rRed = 0, rBlue = 0;

    /**
     * Meters and Radians
     */
    public FudgeablePose(double x, double y, double angle) {
        super(x, y, new Rotation2d(angle));
        fTrans = new FudgeableTranslation(x, y);
    }
    public static FudgeablePose newWithInches(double x, double y, double angle) {
        return new FudgeablePose(Units.inchesToMeters(x), Units.inchesToMeters(y), angle);
    }

    // This is for Zach
    public static FudgeablePose newWithInchesAndDegreesForZach(double x, double y, double angle) {
        return new FudgeablePose(Units.inchesToMeters(x), Units.inchesToMeters(y), Units.degreesToRadians(angle));
    }

    /**
     * Meters and Radians
     */
    public FudgeablePose(FudgeableTranslation translation, double angle) {
        super(translation, new Rotation2d(angle));

        fTrans = translation;
    }

    public FudgeablePose setRotation(double angle) {
        return new FudgeablePose(fTrans, angle);
    }

    public FudgeablePose withXFudge(double blue, double red) {
        fTrans.withXFudge(blue, red);
        return this;
    }

    public FudgeablePose withYFudge(double blue, double red) {
        fTrans.withYFudge(blue, red);
        return this;
    }

    public FudgeablePose withXFudgeInches(double blue, double red) {
        fTrans.withXFudgeInches(blue, red);
        return this;
    }

    public FudgeablePose withYFudgeInches(double blue, double red) {
        fTrans.withYFudgeInches(blue, red);
        return this;
    }

    public FudgeablePose withRotFudge(double blue, double red) {
        rRed = red;
        rBlue = blue;
        return this;
    }

    public Pose2d getBluePose() {
        return new Pose2d(fTrans.getBlueTranslation(), getRotation().plus(new Rotation2d(rBlue)));
    }

    public Pose2d getRedPose() {
        return new Pose2d(fTrans.getRedTranslation(), getRotation().plus(new Rotation2d(rRed)));
    }

    public FudgeablePose getMirrored() {
        FudgeablePose mirrored = new FudgeablePose(fTrans, Math.PI + getRotation().getRadians());
        mirrored.fTrans = fTrans;
        mirrored.rRed = rRed;
        mirrored.rBlue = rBlue;
        return mirrored;
    }

    public Pose2d getPose(DriverStation.Alliance alliance) {
        return
                alliance == DriverStation.Alliance.Blue ?
                        getBluePose() :
                        getRedPose();
    }

    public Translation2d getFudgedTranslation(DriverStation.Alliance alliance) {
        return fTrans.getTranslation(alliance);
    }
}
