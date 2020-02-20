package frc.robot.utils;

import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.Map;
import java.util.Properties;
import java.util.logging.Level;
import java.util.logging.Logger;

import edu.wpi.first.wpilibj.Filesystem;

public class PropertyFiles {
  private static Logger logger = Logger.getLogger(PropertyFiles.class.getName());

  public static Properties loadProperties(String name, boolean loadDefault) {
    String location = "/" + name + ".properties";
    Properties properties = null;
    Properties defaultProperties = new Properties();
    try {
      InputStream deployStream = new FileInputStream(Filesystem.getDeployDirectory() + location);
      defaultProperties.load(deployStream);
      properties = new Properties(defaultProperties);
      if (loadDefault) {
        InputStream robotStream = new FileInputStream(Filesystem.getOperatingDirectory() + location);
        properties.load(robotStream);
      }
      logger.info("successfully loadded " + name);
    } catch (IOException e) {
      logger.log(Level.SEVERE, "failed to load " + name, e);
    }
    return properties;

  }

  public static Properties loadProperties(String name) {
    return PropertyFiles.loadProperties(name, false);
  }

  public static void saveProperties(Properties properties, Map<String, Object> values, String name) {
    properties.clear();
      try {
        String location = "/" + name + ".properties";
        for (String key : values.keySet()) {
          properties.setProperty(key, values.get(key).toString());
        }
        OutputStream outputStream = new FileOutputStream(Filesystem.getOperatingDirectory() + location);
        properties.store(outputStream, "saved properties");
        logger.info("successfully saved " + name);
      } catch(IOException e) {
        logger.log(Level.SEVERE, "failed to save " + name, e);
      }
      properties = PropertyFiles.loadProperties(name);
  }

}