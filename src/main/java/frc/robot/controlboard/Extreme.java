/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.controlboard;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class Extreme {
  public final int m_stickYAxis = 1;

  private final Joystick m_joystick;

  public final Button trigger;
  public final Button sideButton;
  public final Button joystickBottomLeft;
  public final Button joystickBottomRight;
  public final Button joystickTopLeft;
  public final Button joystickTopRight;
  public final Button baseFrontLeft;
  public final Button baseFrontRight;
  public final Button baseMiddleLeft;
  public final Button baseMiddleRight;
  public final Button baseBackLeft;
  public final Button baseBackRight;

  public Extreme(int port) {
    m_joystick = new Joystick(port);

    trigger = new JoystickButton(m_joystick, 1);
    sideButton = new JoystickButton(m_joystick, 2);
    joystickBottomLeft = new JoystickButton(m_joystick, 3);
    joystickBottomRight = new JoystickButton(m_joystick, 4);
    joystickTopLeft = new JoystickButton(m_joystick, 5);
    joystickTopRight = new JoystickButton(m_joystick, 6);
    baseFrontLeft = new JoystickButton(m_joystick, 7);
    baseFrontRight = new JoystickButton(m_joystick, 8);
    baseMiddleLeft = new JoystickButton(m_joystick, 9);
    baseMiddleRight = new JoystickButton(m_joystick, 10);
    baseBackLeft = new JoystickButton(m_joystick, 11);
    baseBackRight = new JoystickButton(m_joystick, 12);
  }

  public double getStickY() {
    return m_joystick.getRawAxis(m_stickYAxis);
  }
}
