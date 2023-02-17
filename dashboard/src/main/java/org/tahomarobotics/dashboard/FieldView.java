/**
 * Copyright 2022 Tahoma Robotics - http://tahomarobotics.org - Bear Metal 2046 FRC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including without
 * limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
 * Software, and to permit persons to whom the Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions
 * of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
 * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 */
package org.tahomarobotics.dashboard;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import javafx.application.Platform;
import javafx.collections.ObservableList;
import javafx.geometry.Rectangle2D;
import javafx.scene.Node;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;
import javafx.scene.image.PixelReader;
import javafx.scene.image.WritableImage;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import javafx.scene.shape.*;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Scale;
import javafx.scene.transform.Translate;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/**
 * Pane displaying the field, robot and trajectories. All distances are specified
 * in meters and angles in radians.
 */
public class FieldView extends Pane {
    private static final Logger log = LoggerFactory.getLogger(FieldView.class);

    // Field image and dimensions
    private static final String FIELD_IMAGE_FILE = "/2022-field.png";
    private static final double FIELD_ORIGIN_X_PIXEL = 74;
    private static final double FIELD_ORIGIN_Y_PIXEL = 50;

    private static final double FIELD_LENGTH_PIXEL_X = 1700;
    private static final double FIELD_WIDTH_PIXEL_Y = 850;

    private static final double FIELD_LENGTH_X = 16.4592;
    private static final double FIELD_WIDTH_Y = 8.2296;

    /** Master scale factor, scales the image (and everything else) */
    private static final double MASTER_SCALE_FACTOR = 2.0;
    /** Field scale factor, the field image pixels/meter */
    private static final double FIELD_SCALE_FACTOR = (FIELD_LENGTH_PIXEL_X / FIELD_LENGTH_X)/MASTER_SCALE_FACTOR;

    // Robot image and dimensions
    private static final String ROBOT_IMAGE_FILE = "/RobotTop.png";
    private static final int ROBOT_HEIGHT_PIXEL_Y = 76;
    private static final int ROBOT_WIDTH_PIXEL_X = 90;
    private static final double ROBOT_WIDTH_X = .8715756;
    /** Robot scale factor, the robot image pixels to meters/pixel */
    private static final double ROBOT_IMAGE_SCALE = (ROBOT_WIDTH_X / ROBOT_WIDTH_PIXEL_X);
    private static final double ROBOT_X_OFFSET = ROBOT_IMAGE_SCALE * ROBOT_WIDTH_PIXEL_X / 2;
    private static final double ROBOT_Y_OFFSET = ROBOT_IMAGE_SCALE * ROBOT_HEIGHT_PIXEL_Y / 2;
    /** Used for robot transformations, never rendered. */
    private final Polygon robot = new Polygon();
    private ImageView robotImage;
    private double[] robotPose = new double[] {9.4117414, 3.1165038, .42}; // MID

    // Additional rendering panes.
    private final Polyline slugTrail = new Polyline();
    private final Polyline autoTraj = new Polyline();
    private final Line lookAheadLine = new Line();
    private final Arc lookAheadArc = new Arc();

    /**
     * Constructor, initalizes the field view with the field and robot images
     */
    public FieldView() {
        final NetworkTable table = DashboardNetworkTable.INSTANCE.getSmartDashboard();
        final NetworkTableEntry robotPoseEntry = table.getEntry("Field/Robot");
        final NetworkTableEntry trajEntry = table.getEntry("Field/traj0");
        final NetworkTableEntry autoEntry = table.getEntry("AutonomousChooser/selected");

        // Draw the field
        // Load field image
        ImageView fieldImage;
        try {
            Image image = new Image(Objects.requireNonNull(FieldView.class.getResourceAsStream(FIELD_IMAGE_FILE)));
            PixelReader reader = image.getPixelReader();
            WritableImage newImage = new WritableImage(reader, (int) FIELD_ORIGIN_X_PIXEL, (int) FIELD_ORIGIN_Y_PIXEL,
                    (int) FIELD_LENGTH_PIXEL_X, (int) FIELD_WIDTH_PIXEL_Y);
            fieldImage = new ImageView(newImage);
        } catch (Exception ex) {
            log.error(String.format("Unable to load field image, %s, field will not be rendered.%n", FIELD_IMAGE_FILE));
            fieldImage = new ImageView();
        }

        Rectangle2D rect = new Rectangle2D(0, 0, FIELD_LENGTH_PIXEL_X, FIELD_WIDTH_PIXEL_Y);
        fieldImage.setViewport(rect);
        fieldImage.setSmooth(true);
        fieldImage.setCache(true);
        fieldImage.setFitWidth(FIELD_LENGTH_PIXEL_X / MASTER_SCALE_FACTOR);
        fieldImage.setPreserveRatio(true);

        setMaxSize(FIELD_LENGTH_PIXEL_X, FIELD_WIDTH_PIXEL_Y);
        setPrefSize(FIELD_LENGTH_PIXEL_X, FIELD_WIDTH_PIXEL_Y);
        // Load robot image
        Image rImage;
        try {
            rImage = new Image(Objects.requireNonNull(FieldView.class.getResourceAsStream(ROBOT_IMAGE_FILE)));
            robotImage = new ImageView(rImage);
            // Draw the robot
            drawRobot(robotPose[0], robotPose[1], robotPose[2]);
        } catch (Exception ex) {
            log.error(String.format("Unable to load robot image, %s, robot will not be rendered.%n", ROBOT_IMAGE_FILE));
            robotImage = new ImageView();
        }

        // Autonomous trajectory
        autoTraj.setStroke(Color.YELLOW);
        autoTraj.setStrokeWidth(0.05);
        autoTraj.toFront();
        // Slug trail
        slugTrail.setStroke(Color.ORANGE);
        slugTrail.setStrokeWidth(0.05);
        slugTrail.toFront();

        // Initialize lookahead line
        lookAheadLine.setStroke(Color.GREEN);
        lookAheadLine.setStrokeType(StrokeType.CENTERED);
        lookAheadLine.setStrokeLineCap(StrokeLineCap.BUTT);
        lookAheadLine.setFill(null);

        // Initialize lookahead arc
        lookAheadArc.setType(ArcType.CHORD);
        lookAheadArc.setStroke(Color.CYAN);
        lookAheadArc.setStrokeType(StrokeType.CENTERED);
        lookAheadArc.setStrokeLineCap(StrokeLineCap.BUTT);
        lookAheadArc.setFill(null);

        autoEntry.addListener(t -> {
            // if ends in "No-Op Auto" clear
            String selectedAuto = t.getEntry().getValue().getString();
            if (selectedAuto.endsWith("No-Op Auto")) {
                drawTraj(List.of(0.0,0.0,0.0,0.0));
            }
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        robotPoseEntry.addListener(t -> {
            robotPose = t.getEntry().getDoubleArray((double[]) null);
            drawRobot(robotPose[0], robotPose[1], robotPose[2]);
            drawSlugTrail(robotPose[0], robotPose[1]);
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        trajEntry.addListener(event -> {
            Platform.runLater(() -> {
                ArrayList<Double> trajectoryPoints = new ArrayList<>();
                Double[] trajArr = event.getEntry().getDoubleArray(new Double[]{});
                extractTrajectory(trajArr, trajectoryPoints);

                // add additional trajectories
                boolean go = true;
                for (int t = 1; go; t++) {
                    NetworkTableEntry trajXEntry = table.getEntry("Field/traj" + t);
                    Double[] trajXArr = trajXEntry.getDoubleArray(new Double[]{});
                    if (trajXArr.length > 0) {
                        extractTrajectory(trajXArr, trajectoryPoints);
                    } else {
                        go = false;
                    }
                }
                drawTraj(trajectoryPoints);
            });}, EntryListenerFlags.kUpdate | EntryListenerFlags.kNew);
        robotImage.toFront();

        // Add all the children
        getChildren().addAll(fieldImage, autoTraj, slugTrail, lookAheadLine, lookAheadArc, robotImage);
    }

    private void extractTrajectory(Double[] trajXArr, ArrayList<Double> trajectoryPoints) {
        for (int i = 0; i < trajXArr.length; i += 3) {
            trajectoryPoints.add(trajXArr[i]);
            trajectoryPoints.add(trajXArr[i + 1]);
        }
    }

    /**
     * Draw the robot on the field.
     * @param x robot location x coordinate in meters
     * @param y robot location y coordinate in meters
     * @param hdg robot heading in radians, 0 is horizontal amd right of center
     */
    private void drawRobot(final double x, final double y, final double hdg) {
        Platform.runLater(() -> {
            // Field transforms
            Scale scale = new Scale(FIELD_SCALE_FACTOR, -FIELD_SCALE_FACTOR);
            Translate translate = new Translate(0 + x, -FIELD_WIDTH_Y + y);
            Rotate rotate = new Rotate(hdg);
            robot.getTransforms().clear();
            robot.getTransforms().addAll(scale, translate, rotate);
            // Robot image transforms
            Scale imageScale = new Scale(ROBOT_IMAGE_SCALE, ROBOT_IMAGE_SCALE);
            Translate imageTranslate = new Translate(-ROBOT_X_OFFSET, -ROBOT_Y_OFFSET);
            robotImage.getTransforms().clear();
            robotImage.getTransforms().addAll(scale, translate, rotate, imageTranslate, imageScale);
        });
    }

    private void drawTraj(List<Double> points) {
        autoTraj.getPoints().clear();
        if (points.size() > 0) {
            autoTraj.getPoints().addAll(points);
        } else {
            autoTraj.getPoints().addAll(0.0,0.0,0.0,0.0);
        }
        applyFieldTransforms(autoTraj);
    }

        private void drawPolyLine(Polyline polyLine, double x, double y) {
        polyLine.getPoints().addAll(x, y);
        applyFieldTransforms(polyLine);
    }

    private void drawSlugTrail(double x, double y) {
        drawPolyLine(slugTrail, x, y);

        if(slugTrail.getPoints().size() > 1000){
            slugTrail.getPoints().remove(0);
            slugTrail.getPoints().remove(0);
        }
    }

    void resetSlugTrail() {
        Platform.runLater(() -> {
            ObservableList<Double> points = slugTrail.getPoints();
            // Maintain the end point
            double x = points.get(points.size()-2);
            double y = points.get(points.size()-1);
            slugTrail.getPoints().clear();
            drawSlugTrail(x, y);
        });
    }

    void enableSlugTrail(final boolean enabled) {
        Platform.runLater(() -> slugTrail.setVisible(enabled));
    }

    /**
     * Applies the standard field transforms to the provided mode, scales and
     * inverts the Y axis.
     * @param node the node to apply the transforms to
     */
    private void applyFieldTransforms(final Node node) {
        Scale scale = new Scale(FIELD_SCALE_FACTOR, -FIELD_SCALE_FACTOR);
        Translate translate = new Translate(0, -FIELD_WIDTH_Y);

        node.getTransforms().clear();
        node.getTransforms().addAll(scale, translate);
    }

    /**
     * Draws and open arch on the field.
     *
     * @param x the center point x coordinate in meters
     * @param y the center point y coordinate in meters
     * @param r the radius in meters
     * @param startAngle the starting angle in radians, 0 is horizontal amd right of center
     * @param arcAngle the arc angle (counterclockwise)
     */
    private void drawArc(final double x, final double y, final double r, final double startAngle, final double arcAngle) {
        lookAheadArc.setCenterX(x);
        lookAheadArc.setCenterY(y);
        lookAheadArc.setRadiusX(r);
        lookAheadArc.setRadiusY(r);
        lookAheadArc.setStartAngle(Math.toDegrees(-startAngle));
        lookAheadArc.setLength(Math.toDegrees(-arcAngle));
        lookAheadArc.setStrokeWidth(.05);
        lookAheadArc.setType(ArcType.OPEN);
        applyFieldTransforms(lookAheadArc);
    }

    /**
     * Draws a line on the field.
     * @param startX starting point x coordinate
     * @param startY starting point y coordinate
     * @param endX end point x coordinator
     * @param endY end point y coordinator
     */
    private void drawLine(double startX, double startY, double endX, double endY) {
        lookAheadLine.setStartX(startX);
        lookAheadLine.setStartY(startY);
        lookAheadLine.setEndX(endX);
        lookAheadLine.setEndY(endY);
        lookAheadLine.setStrokeWidth(.05);
        applyFieldTransforms(lookAheadLine);
    }

}
