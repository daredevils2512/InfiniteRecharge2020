/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.logging.*;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.sensors.PhotoEye;

public class Magazine extends PropertySubsystem {
  public static class MagazineMap {
    public int runMotorID = -1;
    public int photoEyeChannel = -1;
  }
  
  private final NetworkTable m_networkTable;
  private final NetworkTableEntry m_directionReversedEntry;
  private final NetworkTableEntry m_powerCellCountEntry;
  
  private boolean m_photoEyeEnabled;
  private final PhotoEye m_photoEye;

  private final WPI_TalonSRX m_runMotor;
  
  private final int ticksPerBall = 0;
  private final double arbitraryFeedForward = 0;

  private int m_powerCellCount;
  private boolean m_powerCellPreviouslyDetectedFront;
  private boolean m_powerCellPreviouslyDetectedBack;

  /**
   * Creates a new magazine
   */
  public Magazine(MagazineMap magazineMap) {
    super(Magazine.class);

    m_networkTable = NetworkTableInstance.getDefault().getTable(getName());
    m_directionReversedEntry = m_networkTable.getEntry("Direction reversed");
    m_powerCellCountEntry = m_networkTable.getEntry("Power cell count");

    m_runMotor = new WPI_TalonSRX(magazineMap.runMotorID);
    m_runMotor.setInverted(InvertType.InvertMotorOutput);

    m_photoEyeEnabled = Boolean.parseBoolean(m_properties.getProperty("photoEyeEnabled"));

    m_photoEye = m_photoEyeEnabled ? new PhotoEye(magazineMap.photoEyeChannel) : null;
  }

  @Override
  public void periodic() {
    if (m_photoEyeEnabled) {
      updatePowerCellCount();
      m_powerCellPreviouslyDetectedFront = getPowerCellDetectedFront();
      m_powerCellPreviouslyDetectedBack = getPowerCellDetectedBack();
    }

    m_directionReversedEntry.setBoolean(getDirectionReversed());
    m_powerCellCountEntry.setNumber(getPowerCellCount());
  }

  public boolean getPowerCellDetectedFront() {
    if (m_photoEyeEnabled) {
      if (m_photoEye.get())
        m_logger.fine("power cell detected front");
      return m_photoEye.get();
    } else {
      return false;
    }
  }

  public boolean getPowerCellDetectedBack() {
    return false;
  }

  public int getPowerCellCount() {
    return m_powerCellCount;
  }

  public void setBallsInMag(int set) {
    m_powerCellCount = set;
  }

  public void resetBallCount() {
    setBallsInMag(0);
  }

  // might be temporary
  public void updatePowerCellCount() {
    int deltaCount = 0;
    if (!getPowerCellDetectedFront() && m_powerCellPreviouslyDetectedFront)
      deltaCount++;
    if (!getPowerCellDetectedBack() && m_powerCellPreviouslyDetectedBack)
      deltaCount--;
    if (getDirectionReversed())
      deltaCount = -deltaCount; // Counting direction is reversed if the magazine is being run backwards

    int newCount = m_powerCellCount + deltaCount;
    if (newCount < 0)
      m_logger.warning("Power cell count exceeded lower bounds");
    else if (newCount > 3)
      m_logger.warning("Power cell count exceeded upper bounds");

    m_powerCellCount = MathUtil.clamp(newCount, 0, 3);
    if (deltaCount != 0)
      m_logger.log(Level.FINER, "power cell count", m_powerCellCount);
  }

  public boolean getDirectionReversed() {
    return m_runMotor.getMotorOutputPercent() < 0;
  }

  public void setSpeed(double speed) {
    m_runMotor.set(ControlMode.PercentOutput, speed);
  }

  public void feedBalls(int amount) {
    m_runMotor.set(ControlMode.MotionMagic, amount * ticksPerBall, DemandType.ArbitraryFeedForward,
        arbitraryFeedForward);
  }

  @Override
  protected Map<String, Object> getValues() {
    return null;
  }
}
