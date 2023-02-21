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
package org.openfx;

import javafx.application.Application;
import javafx.collections.ObservableList;
import javafx.geometry.Rectangle2D;
import javafx.scene.Scene;
import javafx.scene.control.*;
import javafx.scene.layout.HBox;
import javafx.scene.layout.Priority;
import javafx.scene.layout.VBox;
import javafx.stage.Screen;
import javafx.stage.Stage;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.dashboard.*;

import java.util.prefs.Preferences;

public class Dashboard extends Application {
    static {
        System.setProperty("log4j2.contextSelector",
                "org.apache.logging.log4j.core.async.AsyncLoggerContextSelector");
    }
    private static final Logger log = LoggerFactory.getLogger(Dashboard.class);

    public static final String TEAM = "2046";
    public static final String SIMULATION_IP = "127.0.0.1";
    public static final String ROBOT_IP = "10." + TEAM.substring(0, 2) + "." + TEAM.substring(2) + ".2"; // 10.20.46.2

    private static final double  WINDOW_PERCENT = 0.80;
    private static final String WINDOW_POSITION_X = "Window_Position_X";
    private static final String WINDOW_POSITION_Y = "Window_Position_Y";
    private static final String WINDOW_WIDTH = "Window_Width";
    private static final String WINDOW_HEIGHT = "Window_Height";
    private static final String NODE_NAME = "Dash-board";

    @Override
    public void start(Stage primaryStage) {
        primaryStage.setTitle("Bear Scope");
        Operator operator = new Operator();
        // setup Menu
        final Menu fileMenu = new Menu("File");
        MenuItem quitMenuItem = new MenuItem("Quit");
        quitMenuItem.setOnAction((event) -> System.exit(0));
        fileMenu.getItems().add(quitMenuItem);
        final Menu optionsMenu = new Menu("Options");
        MenuBar menuBar = new MenuBar();
        CheckMenuItem enableSlugTrail = new CheckMenuItem("Slug Trail");
        enableSlugTrail.setSelected(false);
        operator.enableSlugTrail(false);
        enableSlugTrail.setOnAction(e -> operator.enableSlugTrail(enableSlugTrail.isSelected()));
        MenuItem clearSlugTrail = new MenuItem("Clear Slug Trail");
        clearSlugTrail.setOnAction(e -> operator.clearSlugTrail());

        optionsMenu.getItems().addAll(enableSlugTrail,clearSlugTrail);
        menuBar.getMenus().addAll(fileMenu, optionsMenu);

        Preferences pref = Preferences.userRoot().node(NODE_NAME);
        Rectangle2D screenSize = Screen.getPrimary().getBounds();
        primaryStage.setWidth(pref.getDouble(WINDOW_WIDTH, screenSize.getWidth() * WINDOW_PERCENT));
        primaryStage.setHeight(pref.getDouble(WINDOW_HEIGHT,screenSize.getHeight() * WINDOW_PERCENT));
        primaryStage.setX(pref.getDouble(WINDOW_POSITION_X, (screenSize.getWidth() - primaryStage.getWidth())/2));
        primaryStage.setY(pref.getDouble(WINDOW_POSITION_Y,(screenSize.getHeight() - primaryStage.getHeight())/2));

        // Create tab pane and tabs
        final TabPane tabPane = new TabPane();
        ObservableList<Tab> tabs = tabPane.getTabs();
        tabs.add(new Tab("Operator", operator));
        tabs.add(new Tab("Paw Position", new PawPosition()));
        tabs.add(new Tab("Paw Velocity", new PawVelocity()));
        tabs.add(new Tab("Arm", new Arm()));
        tabs.add(new Tab("Arm Angles", new ArmAngles()));

        // combine and show
        VBox vBox = new VBox();
        vBox.getChildren().addAll(menuBar, tabPane);
        vBox.setFillWidth(true);
        VBox.setVgrow(tabPane, Priority.ALWAYS);
        HBox.setHgrow(tabPane, Priority.ALWAYS);
        Scene scene = new Scene(vBox);
        primaryStage.setScene(scene);
        primaryStage.show();

        log.info("Dashboard successfully configured.");
    }

    public static void main(String[] args) {
        launch();
    }
}