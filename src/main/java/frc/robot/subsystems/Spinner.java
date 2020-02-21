/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.logging.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.ColorSensor;
import frc.robot.sensors.ColorSensor.ColorDetect;
import frc.robot.subsystems.interfaces.ISpinner;

public class Spinner extends SubsystemBase implements ISpinner{
  private ColorSensor m_colorSensor = new ColorSensor();
  private static Logger logger = Logger.getLogger(Spinner.class.getName());

  /**
   * Creates a new Spinner.
   */
  public Spinner() {

  }

  @Override
  public void periodic() {

    Color detectedColor = m_colorSensor.getColor();
    SmartDashboard.putNumber("confidence", m_colorSensor.getColorMatch().confidence);
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putString("Color", m_colorSensor.getColorMatchDetect().name());
    // This method will be called once per scheduler run
  }

  @Override
  public ColorDetect getCurrentColor() {
    logger.log(Level.FINE, "current color =", m_colorSensor.getColorMatch());
    return m_colorSensor.getColorMatchDetect();
  }

  @Override
  public void setExtended(boolean wantsExtended){
    // Actuate somehow
  }

  @Override
  public void run(double speed) {
    // Spins somehow
  }

}
