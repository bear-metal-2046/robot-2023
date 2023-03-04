package org.tahomarobotics.robot.util;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.*;

import java.util.Map;

public class Shufflebear {
    private static final Shufflebear INSTANCE = new Shufflebear();
    private Shufflebear(){}

    public Shufflebear getInstance() {
        return INSTANCE;
    }


    public static SimpleWidget addNumber(String paneName, String name, double value, int x, int y, int sizeX, int sizeY, String... layouts) {
        if (layouts.length > 0) {
            ShuffleboardLayout currentLayout = Shuffleboard.getTab(paneName).getLayout(layouts[0]);
            for (int i = 1; i < layouts.length; i++) {
                currentLayout = currentLayout.getLayout(layouts[i]);
            }
            return currentLayout.add(name, value).withPosition(x, y).withSize(sizeX, sizeY);
        } else {
            return Shuffleboard.getTab(paneName).add(name, value).withPosition(x, y).withSize(sizeX, sizeY);
        }
    }


    public static SimpleWidget addString(String paneName, String name, String value, int x, int y, int sizeX, int sizeY, String... layouts) {
        if (layouts.length > 0) {
            ShuffleboardLayout currentLayout = Shuffleboard.getTab(paneName).getLayout(layouts[0]);
            for (int i = 1; i < layouts.length; i++) {
                currentLayout = currentLayout.getLayout(layouts[i]);
            }
            return currentLayout.add(name, value).withPosition(x, y).withSize(sizeX, sizeY);
        } else {
            return Shuffleboard.getTab(paneName).add(name, value).withPosition(x, y).withSize(sizeX, sizeY);
        }
    }


    public static ComplexWidget addSendable(String paneName, String name, Sendable value, int x, int y, int sizeX, int sizeY, String... layouts) {
        if (layouts.length > 0) {
            ShuffleboardLayout currentLayout = Shuffleboard.getTab(paneName).getLayout(layouts[0]);
            for (int i = 1; i < layouts.length; i++) {
                currentLayout = currentLayout.getLayout(layouts[i]);
            }
            return currentLayout.add(name, value).withPosition(x, y).withSize(sizeX, sizeY);
        } else {
            return Shuffleboard.getTab(paneName).add(name, value).withPosition(x, y).withSize(sizeX, sizeY);
        }
    }



    public static SimpleWidget addBool(String paneName, String name, boolean value, int x, int y, int sizeX, int sizeY, String... layouts) {
        if (layouts.length > 0) {
            ShuffleboardLayout currentLayout = Shuffleboard.getTab(paneName).getLayout(layouts[0]);
            for (int i = 1; i < layouts.length; i++) {
                currentLayout = currentLayout.getLayout(layouts[i]);
            }
            return currentLayout.add(name, value).withPosition(x, y).withSize(sizeX, sizeY).withWidget(BuiltInWidgets.kToggleSwitch);
        } else {
            return Shuffleboard.getTab(paneName).add(name, value).withPosition(x, y).withSize(sizeX, sizeY).withWidget(BuiltInWidgets.kToggleSwitch);
        }
    }


    public static ShuffleboardLayout makeLayout(String paneName, String name, int x, int y, int sizeX, int sizeY, String layout, Map<String, Object> properties, String... layouts) {
        if (layouts.length > 0) {
            ShuffleboardLayout currentLayout = Shuffleboard.getTab(paneName).getLayout(layouts[0]);
            for (int i = 1; i < layouts.length; i++) {
                currentLayout = currentLayout.getLayout(layouts[i]);
            }
            return currentLayout.getLayout(name, layout).withPosition(x, y).withSize(sizeX, sizeY).withProperties(properties);
        } else {
            return Shuffleboard.getTab(paneName).getLayout(name, layout).withPosition(x, y).withSize(sizeX, sizeY).withProperties(properties);
        }
    }


    public static ShuffleboardLayout makeLayout(String paneName, String name, int x, int y, int sizeX, int sizeY, LayoutType layout, Map<String, Object> properties, String... layouts) {
        if (layouts.length > 0) {
            ShuffleboardLayout currentLayout = Shuffleboard.getTab(paneName).getLayout(layouts[0]);
            for (int i = 1; i < layouts.length; i++) {
                currentLayout = currentLayout.getLayout(layouts[i]);
            }
            return currentLayout.getLayout(name, layout).withPosition(x, y).withSize(sizeX, sizeY).withProperties(properties);
        } else {
            return Shuffleboard.getTab(paneName).getLayout(name, layout).withPosition(x, y).withSize(sizeX, sizeY).withProperties(properties);
        }
    }

    public static ShuffleboardLayout makeLayout(String paneName, String name, int x, int y, int sizeX, int sizeY, LayoutType layout, String... layouts) {
        if (layouts.length > 0) {
            ShuffleboardLayout currentLayout = Shuffleboard.getTab(paneName).getLayout(layouts[0]);
            for (int i = 1; i < layouts.length; i++) {
                currentLayout = currentLayout.getLayout(layouts[i]);
            }
            return currentLayout.getLayout(name, layout).withPosition(x, y).withSize(sizeX, sizeY);
        } else {
            return Shuffleboard.getTab(paneName).getLayout(name, layout).withPosition(x, y).withSize(sizeX, sizeY);
        }
    }

    public static ShuffleboardTab getPane(String name) {
        return Shuffleboard.getTab(name);
    }

}
