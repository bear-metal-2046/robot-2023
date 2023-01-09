package org.tahomarobotics.robot.configuration;

import org.simpleyaml.configuration.file.YamlConfiguration;
import org.tahomarobotics.robot.util.LoggerManager;

import java.io.File;
import java.io.IOException;

public class RobotConfig {

    private YamlConfiguration config;

    public RobotConfig(File file) {
        loadConfig(file);
    }

    private void loadConfig(File file) {
        try {
            config.load(file);
        } catch (IOException e) {
            LoggerManager.error("Failed to load configuration into SimpleYAML Object, is there a configuration present?");
            e.printStackTrace();
        }
        if(!checkValid()) {
            LoggerManager.error("Killing Runtime, cannot proceed with invalid configuration at risk of damage to robot.");
            Runtime.getRuntime().exit(0);
        }
    }

    private boolean checkValid() {
        boolean valid = config.getBoolean("is_this_a_config");
        int version = config.getInt("version");
        if(!valid || version == 0) {
            LoggerManager.warn("Invalid Configuration, check to make sure proper values are present. (version/is_this_a_config)");
            return false;
        } else {
            return true;
        }
    }
}
