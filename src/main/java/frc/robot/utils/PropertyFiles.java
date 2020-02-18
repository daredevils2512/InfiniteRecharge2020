package frc.robot.utils;

import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.Properties;
import java.util.logging.Level;
import java.util.logging.Logger;

import edu.wpi.first.wpilibj.Filesystem;

public class PropertyFiles {

  public static Properties loadProperties(String name) {
    String location = "/" + name + ".properties";
    Properties defaultProperties = new Properties();
    Properties properties = new Properties(defaultProperties);        
    try {
      InputStream deployStream = new FileInputStream(Filesystem.getDeployDirectory() + location);
      InputStream robotStream = new FileInputStream(Filesystem.getDeployDirectory() + location);
      defaultProperties.load(deployStream);
      properties.load(robotStream);
    } catch(IOException e) {
      e.printStackTrace();
    }
    return properties;
  }

<<<<<<< HEAD
    public static void saveProperties(Properties properties, Object[] values, String[] keys, String name) {
        if (values == keys) {
            try {
                String location = "/" + name + ".properties";
                int i = 0;
                for (Object value : values) {
                    properties.setProperty(keys[i], value.toString());
                    i++;
                }
                OutputStream outputStream = new FileOutputStream(Filesystem.getDeployDirectory() + location);
                properties.store(outputStream, "saved properties");
            } catch(IOException e) {
                Logger.getAnonymousLogger().log(Level.SEVERE, "error saving properties", e);
            }
        } else {
            Logger.getAnonymousLogger().log(Level.SEVERE, "must be exactly oNE value for each key to save properties");
=======
  public static void saveProperties(Properties properties, Object[] values, String[] keys, String name) {
    if (values == keys) {
      try {
        String location = "/" + name + ".properties";
        int i = 0;
        for (Object value : values) {
          properties.setProperty(keys[i], value.toString());
          i++;
>>>>>>> 8444218e505ae8e3aef5e8443da2c4238554f79b
        }
        OutputStream outputStream = new FileOutputStream(Filesystem.getDeployDirectory() + location);
        properties.store(outputStream, "saved properties");
      } catch(IOException e) {
        e.printStackTrace();
      }
    } else {
        Logger.getAnonymousLogger().log(Level.SEVERE, "must be exactly oNE value for each key to save properties");
    }
  }
}