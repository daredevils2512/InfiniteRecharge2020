/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.controlboard;

import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.lang.reflect.Field;
import java.util.Properties;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.utils.PropertyFiles;

public class ControlBoard {
  private final int m_xboxPort = 0;
  private final int m_extremePort = 1;
  private final int m_buttonBoxPort = 2;
  private final Properties m_properties;

  public final Xbox xbox = new Xbox(m_xboxPort);
  public final Extreme extreme = new Extreme(m_extremePort);
  public final ButtonBox buttonBox = new ButtonBox(m_buttonBoxPort);

  public ControlBoard() {
    m_properties = PropertyFiles.loadProperties(this.getClass().getSimpleName().toLowerCase());
  }


  /**
   * mainly for testing
   * @param propertiesstring a string that pretends to be a file for testing
   */
  ControlBoard(String propertiesString) {
    m_properties = PropertyFiles.loadFromString(propertiesString);
}

  public JoystickButton getButton(String propertiesKey) {
    String propertiesValue = m_properties.getProperty(propertiesKey);
    propertiesValue = propertiesValue.trim();
    Class<? extends ControlBoard> cls = this.getClass();
    String[] splitValue = propertiesValue.split("\\.");
    String joystick = splitValue[0];
    String value = splitValue[1];
    JoystickButton button;
    try {
      Object obj = cls.getDeclaredField(joystick).get(this);
      Field field = obj.getClass().getDeclaredField(value);
      button = (JoystickButton) field.get(obj);
    } catch(IllegalAccessException | NoSuchFieldException e) {
      button = null;
      e.printStackTrace();
    }
    return button;
  }
}
