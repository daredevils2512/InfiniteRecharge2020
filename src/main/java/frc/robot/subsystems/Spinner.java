/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.logging.Level;
import java.util.logging.Logger;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.ColorSensor;
import frc.robot.sensors.ColorSensor.ColorDetect;

public class Spinner extends SubsystemBase {
  private final Logger m_logger;

  private final NetworkTable m_networkTable;
  private final NetworkTableEntry m_detectedColorEntry;
  private final NetworkTableEntry m_confidenceEntry;

  private ColorSensor m_colorSensor;

  /**
   * 
   * Creates a new Spinner.
   */
  public Spinner() {
    m_logger = Logger.getLogger(getName());

    m_networkTable = NetworkTableInstance.getDefault().getTable(getName());
    m_detectedColorEntry = m_networkTable.getEntry("Detected color");
    m_confidenceEntry = m_networkTable.getEntry("Confidence");

    m_colorSensor = new ColorSensor();
  }

  @Override
  public void periodic() {
    ColorDetect color = m_colorSensor.getColorMatchDetect();
    m_detectedColorEntry.setString(color.name());
    m_confidenceEntry.setNumber(m_colorSensor.getColorMatch().confidence);
  }

  public ColorDetect getCurrentColor() {
    m_logger.log(Level.FINE, "current color: ", m_colorSensor.getColorMatch());
    return m_colorSensor.getColorMatchDetect();
  }

  public void setExtended(boolean wantsExtended){
    // Actuate somehow
  }

  public void run(double speed) {
    // Spins somehow
  }
}
