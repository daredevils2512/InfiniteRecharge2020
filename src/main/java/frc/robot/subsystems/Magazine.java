/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.logging.*;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.Map;
import java.util.Properties;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.sensors.PhotoEye;
import frc.robot.utils.PropertyFiles;

public class Magazine extends PropertySubsystem {

  private final NetworkTable m_networkTable;
  private final NetworkTableEntry m_directionReversedEntry;
  private final NetworkTableEntry m_powerCellCountEntry;

  private boolean m_photoEyeEnabled;
  private final int m_frontPhotoEyeChannel = -1;
  private final int m_backPhotoEyeChannel = -1;
  private final PhotoEye m_frontPhotoEye; // Photo eye closest to the intake
  private final PhotoEye m_backPhotoEye; // Photo eye closest to the queue

  private final int m_magazineRunMotorID;
  private final WPI_TalonSRX m_magazineRunMotor;

  private final int ticksPerBall = 0;
  private final double arbitraryFeedForward = 0;

  private int m_powerCellCount;
  private boolean m_powerCellPreviouslyDetectedFront;
  private boolean m_powerCellPreviouslyDetectedBack;

  /**
   * Creates a new magazine
   */
  public Magazine() {
    super(Magazine.class.getSimpleName());

    m_networkTable = NetworkTableInstance.getDefault().getTable(getName());
    m_directionReversedEntry = m_networkTable.getEntry("Direction reversed");
    m_powerCellCountEntry = m_networkTable.getEntry("Power cell count");
    m_magazineRunMotorID = Integer.parseInt(properties.getProperty("magazineRunMotorID"));
    m_photoEyeEnabled = Boolean.parseBoolean(properties.getProperty("photoEyeEnabled"));

    if (m_photoEyeEnabled) {
      m_frontPhotoEye = new PhotoEye(m_frontPhotoEyeChannel);
      m_backPhotoEye = new PhotoEye(m_backPhotoEyeChannel);
    } else {
      m_frontPhotoEye = null;
      m_backPhotoEye = null;
    }
    m_magazineRunMotor = new WPI_TalonSRX(m_magazineRunMotorID);
  }

  @Override
  public void periodic() {
    updatePowerCellCount();
    m_powerCellPreviouslyDetectedFront = getPowerCellDetectedFront();
    m_powerCellPreviouslyDetectedBack = getPowerCellDetectedBack();

    m_directionReversedEntry.setBoolean(getDirectionReversed());
    m_powerCellCountEntry.setNumber(getPowerCellCount());
  }

  public boolean getPowerCellDetectedFront() {
    if (m_photoEyeEnabled) {
      if (m_frontPhotoEye.get())
        logger.fine("power cell detected front");
      return m_frontPhotoEye.get();
    } else {
      return false;
    }
  }

  public boolean getPowerCellDetectedBack() {
    if (m_photoEyeEnabled) {
      if (m_backPhotoEye.get())
        logger.fine("power cell detected back");
      return m_backPhotoEye.get();
    } else {
      return false;
    }
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
      logger.log(Level.WARNING, "Power cell count exceeded lower bounds");
    else if (newCount > 3)
      logger.log(Level.WARNING, "Power cell count exceeded upper bounds");

    m_powerCellCount = MathUtil.clamp(newCount, 0, 3);
    if (deltaCount != 0)
      logger.log(Level.FINER, "power cell count", m_powerCellCount);
  }

  public boolean getDirectionReversed() {
    return m_magazineRunMotor.getMotorOutputPercent() < 0;
  }

  public void setSpeed(double speed) {
    m_magazineRunMotor.set(ControlMode.PercentOutput, speed);
  }

  public void feedBalls(int amount) {
    m_magazineRunMotor.set(ControlMode.MotionMagic, amount * ticksPerBall, DemandType.ArbitraryFeedForward,
        arbitraryFeedForward);
  }

  @Override
  protected Map<String, Object> getValues() {
    return null;
  }
}
