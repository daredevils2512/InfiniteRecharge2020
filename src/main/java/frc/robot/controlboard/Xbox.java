/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.controlboard;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class Xbox {
  private final XboxController m_controller;

  public final Button aButton;
  public final Button bButton;
  public final Button xButton;
  public final Button yButton;
  public final Button leftBumper;
  public final Button rightBumper;
  public final Button backButton;
  public final Button startButton;
  public final Button leftStickButton;
  public final Button rightStickButton;
  public final Button leftTrigger;
  public final Button rightTrigger;

  public Xbox(int port) {
    m_controller = new XboxController(port);

    aButton = new JoystickButton(m_controller, 1);
    bButton = new JoystickButton(m_controller, 2);
    xButton = new JoystickButton(m_controller, 3);
    yButton = new JoystickButton(m_controller, 4);
    leftBumper = new JoystickButton(m_controller, 5);
    rightBumper = new JoystickButton(m_controller, 6);
    backButton = new JoystickButton(m_controller, 7);
    startButton = new JoystickButton(m_controller, 8);
    leftStickButton = new JoystickButton(m_controller, 9);
    rightStickButton = new JoystickButton(m_controller, 10);
    leftTrigger = new JoystickButton(m_controller, 11);
    rightTrigger = new JoystickButton(m_controller, 12);
  }

  public double getLeftStickY() {
    return m_controller.getY(Hand.kLeft);
  }

  public double getRightStickX() {
    return m_controller.getX(Hand.kRight);
  }
}
