/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.controlboard;

import java.io.ByteArrayInputStream;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.lang.reflect.Field;
import java.util.Properties;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.utils.PropertyFiles;

public class ControlBoard {
  private final int xboxPort = 0;
  private final int extremePort = 1;
  private final Properties properties;

  public final Xbox xbox = new Xbox(xboxPort);
  public final Extreme extreme = new Extreme(extremePort);

  public ControlBoard() {
    properties = PropertyFiles.loadProperties(this.getClass().getSimpleName().toLowerCase());
  }

  public ControlBoard(String propertiesstring) {
    properties = new Properties();
    try {
      InputStream deployStream = new ByteArrayInputStream(propertiesstring.getBytes());
      properties.load(deployStream);
    } catch(IOException e) {
      e.printStackTrace();
    }
}

  public JoystickButton getButton(String propertiesKey) {
    String propertiesValue = properties.getProperty(propertiesKey);
    Class cls = this.getClass();
    String[] splitValue = propertiesValue.split("\\.");
    String joystick = splitValue[0];
    String value = splitValue[1];
    JoystickButton button;
    try {
      Object obj = cls.getDeclaredField(joystick).get(this);
      Field field = obj.getClass().getDeclaredField(value);
      button = (JoystickButton) field.get(obj);
    } catch(IllegalAccessException e) {
      button = null;
      e.printStackTrace();
    } catch(NoSuchFieldException e) {
      button = null;
      e.printStackTrace();
    }
    return button;
  }
}
