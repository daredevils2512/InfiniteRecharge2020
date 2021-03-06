package frc.robot.utils;

import java.io.ByteArrayInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
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

  public static Properties loadProperties(Properties properties, File propertiesFile) {
    try {
      InputStream inputStream = new FileInputStream(propertiesFile);
      properties.load(inputStream);
    } catch (FileNotFoundException e) {
      logger.log(Level.SEVERE, "Properties file " + propertiesFile.getPath() + " not found!", e);
    } catch (IOException e) {
      logger.log(Level.SEVERE, "Failed to load properties from " + propertiesFile.getPath() + "!", e);
    }

    return properties;
  }

  public static Properties loadProperties(File propertiesFile) {
    Properties properties = new Properties();
    loadProperties(properties, propertiesFile);
    return properties;
  }

  public static Properties loadProperties(File defaultPropertiesFile, File propertiesFile) {
    Properties defaultProperties = new Properties();
    loadProperties(defaultProperties, defaultPropertiesFile);
    Properties properties = new Properties(defaultProperties);
    loadProperties(properties, propertiesFile);
    return properties;
  }

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

  public static Properties loadFromString(String propertiesString) {
    Properties properties = new Properties();
    try {
      InputStream inputStream = new ByteArrayInputStream(propertiesString.getBytes());
      properties.load(inputStream);
    } catch(IOException e) {
      logger.log(Level.WARNING, "failed to load properties string : ", e);
    }
    return properties;
  }

  public static void saveProperties(Properties properties, File propertiesFile) {
    try {
      OutputStream outputStream = new FileOutputStream(propertiesFile);
      properties.store(outputStream, "Saved properties");
    } catch (FileNotFoundException e) {
      logger.log(Level.SEVERE, "Properties file " + propertiesFile.getPath() + " not found!", e);
    } catch (IOException e) {
      logger.log(Level.SEVERE, "Failed to save properties to " + propertiesFile.getPath() + "!", e);
    }
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