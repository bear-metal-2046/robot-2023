/*
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

import javafx.application.Platform;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.geometry.Insets;
import javafx.geometry.Pos;
import javafx.scene.chart.LineChart;
import javafx.scene.chart.NumberAxis;
import javafx.scene.chart.XYChart;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Label;
import javafx.scene.layout.AnchorPane;
import javafx.scene.layout.GridPane;
import javafx.scene.layout.VBox;
import org.tahomarobotics.robot.util.ChartData;

import java.util.ArrayList;
import java.util.stream.Collectors;

public class XYChartPane extends VBox {
    private static final int MAX_HISTORY = 10;

    private final NumberAxis xAxis = new NumberAxis();
    private final NumberAxis yAxis = new NumberAxis();
    private final LineChart<Number, Number> chart = new LineChart<>(xAxis, yAxis);

    private ArrayList<DataCollection> history = new ArrayList<>();
    private int historyCnt = 0;
    private ComboBox<String> historyComboBox;

    private static class DataCollection {
        public final String title;
        public final ChartData chartData;

        public DataCollection(String title, ChartData chartData) {
            this.title = title;
            this.chartData = chartData;
        }
    }

    public XYChartPane() {
        AnchorPane.setTopAnchor(chart, 0.0);
        AnchorPane.setLeftAnchor(chart, 0.0);
        AnchorPane.setRightAnchor(chart, 0.0);
        AnchorPane.setBottomAnchor(chart, 0.0);
        chart.setAnimated(false);
        AnchorPane anchor = new AnchorPane();
        anchor.getChildren().add(chart);
        GridPane historyPane = new GridPane();
        historyPane.setAlignment(Pos.CENTER_LEFT);
        historyPane.setHgap(5);
        historyPane.setVgap(5);
        historyPane.setPadding(new Insets(5, 5, 5, 5));

        Label label = new Label("History");
        historyComboBox = new ComboBox<>();
        historyPane.add(label, 0, 0);
        historyPane.add(historyComboBox, 1, 0);

        historyComboBox.setOnAction((event) -> {
            Platform.runLater(this::renderData);
        });

        getChildren().addAll(historyPane, anchor);
    }

    public void update(final ChartData chartData) {
        updateChart(chartData);
    }

    private void updateChart(final ChartData chartData) {
        historyCnt++;
        while(history.size() >= MAX_HISTORY) {
            history.remove(MAX_HISTORY-1);
        }
        DataCollection sample = new DataCollection("Sample " + historyCnt, chartData);
        history.add(0, sample);
        Platform.runLater(()-> {
            historyComboBox.setItems(history.stream()
                            .map(c -> c.title)
                            .collect(Collectors.toCollection(FXCollections::observableArrayList)));
            historyComboBox.setValue(sample.title);
        });
        Platform.runLater(this::renderData);
    }

    private void renderData() {
        int ndx = historyComboBox.getSelectionModel().getSelectedIndex();

        ChartData chartData = history.get(ndx).chartData;
        int seriesCount = chartData.getSeriesCount();
        chart.setTitle(chartData.getTitle());
        xAxis.setLabel(chartData.getXAxis());
        yAxis.setLabel(chartData.getYAxis());

        chart.getData().clear();

        for (int i = 0; i < seriesCount; i++) {
            ObservableList<XYChart.Data<Number, Number>> seriesData = FXCollections.<XYChart.Data<Number, Number>>observableArrayList();

            for(double[] data : chartData.getData()) {
                seriesData.add(new XYChart.Data<Number, Number>(data[0], data[i+1]));
            }
            XYChart.Series<Number, Number> series = new XYChart.Series<Number, Number>(chartData.getSeriesName(i), seriesData);
            chart.getData().add(series);
        }
    }
}
