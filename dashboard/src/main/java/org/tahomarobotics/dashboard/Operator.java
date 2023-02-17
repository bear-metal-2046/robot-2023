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
import javafx.collections.FXCollections;
import javafx.geometry.Insets;
import javafx.geometry.Pos;
import javafx.scene.control.Alert;
import javafx.scene.control.CheckBox;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Label;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.GridPane;
import javafx.scene.layout.HBox;
import javafx.scene.text.Font;
import org.openfx.Dashboard;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.stream.Collectors;
import java.util.stream.Stream;

public class Operator extends HBox {
    private static final Logger log = LoggerFactory.getLogger(Operator.class);

    private final NetworkTable table;
    private final FieldView fieldView;
    private final ComboBox<String> autoComboBox;

    public Operator() {
        table = DashboardNetworkTable.INSTANCE.getSmartDashboard();

        CheckBox networkButton = new CheckBox("Robot Network");
        networkButton.setSelected(true);
        networkButton.setOnAction((event) -> {
            String selectedIp = networkButton.isSelected() ? Dashboard.ROBOT_IP : Dashboard.SIMULATION_IP;
            System.err.println("selected: " + selectedIp + ", current: " + DashboardNetworkTable.INSTANCE.getHost());
            if (!selectedIp.equals(DashboardNetworkTable.INSTANCE.getHost())) {
                DashboardNetworkTable.INSTANCE.changeHost(selectedIp);
            }
            log.info("Network changed: " + (networkButton.isSelected() ? "robot" : "simulation"));
        });

        // Drop down for autonomous selection
        Label autoLabel = new Label("Autonomous");
        autoComboBox = new ComboBox<>();
        autoComboBox.setOnAction((event) -> {
            String value = autoComboBox.getSelectionModel().getSelectedItem();
            NetworkTableEntry entry = table.getEntry("AutonomousChooser/selected");
            if (entry != null && value != null) {
                table.getEntry("AutonomousChooser/selected").setString(value);
            }
        });

        Label title = new Label("Limelight Distance Offset");
        Label manualOffset = new Label("Empty");
        manualOffset.setFont(new Font(40));
        table.addEntryListener("Manual Offset", (table, key , entry, value, flag) -> {
            Platform.runLater(() ->  manualOffset.setText(Double.toString(value.getDouble())));
        }, EntryListenerFlags.kUpdate | EntryListenerFlags.kNew);

        // Populate autonomous comboBox from SmartDashboard
        listenToChooser(table, autoComboBox, "AutonomousChooser");

        // Layout the settings pane
        GridPane settingsPane = new GridPane();
        settingsPane.setAlignment(Pos.TOP_CENTER);
        settingsPane.setHgap(5);
        settingsPane.setVgap(5);
        settingsPane.setPadding(new Insets(5, 5, 5, 5));

        settingsPane.add(networkButton, 0, 0, 2, 1);
        settingsPane.add(autoLabel, 0, 1);
        settingsPane.add(autoComboBox, 1, 1);
        settingsPane.add(title, 0 , 2);
        settingsPane.add(manualOffset, 1, 2);

        fieldView = new FieldView();

        BorderPane borderPane = new BorderPane();
        borderPane.setLeft(settingsPane);
        borderPane.setRight(fieldView);
        getChildren().add(borderPane);
    }

    private void listenToChooser(NetworkTable t, ComboBox<String> cb, String chooserName) {
        t.addEntryListener(chooserName + "/options",
                (table, key, entry, value, flag) -> {
                    Platform.runLater(() ->
                            cb.setItems(Stream.of(value.getStringArray())
                                    .collect(Collectors.toCollection(FXCollections::observableArrayList))));
                },
                EntryListenerFlags.kUpdate | EntryListenerFlags.kNew);

        t.addEntryListener(chooserName + "/selected",
                (table, key, entry, value, flag) -> {
                    Platform.runLater(() ->
                            cb.setValue(value.getString()));
                },
                EntryListenerFlags.kUpdate | EntryListenerFlags.kNew);
    }

    public void enableSlugTrail(final boolean enable) {
        fieldView.enableSlugTrail(enable);
    }

    public void clearSlugTrail() {
        fieldView.resetSlugTrail();
    }

    private void postError() {
        Alert alert = new Alert(Alert.AlertType.ERROR);
        alert.setTitle("Autonomous Selection Error");
        alert.setHeaderText("Missing autonomous route");
        alert.setContentText("Autonomous routing information not available");
        alert.showAndWait();
    }
}
