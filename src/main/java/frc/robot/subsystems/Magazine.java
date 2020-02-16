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

public class Magazine extends SubsystemBase {
  private static Logger logger = Logger.getLogger(Magazine.class.getName());
  private final Properties properties;
  private static final String PROPERTIES_NAME = "/magazine.properties";

  private final NetworkTable m_networkTable;
  private final NetworkTableEntry m_directionReversedEntry;
  private final NetworkTableEntry m_powerCellCountEntry;
  
  private final int m_frontPhotoEyeChannel;
  private final int m_backPhotoEyeChannel;
  private final PhotoEye m_frontPhotoEye; // Photo eye closest to the intake
  private final PhotoEye m_backPhotoEye; // Photo eye closest to the queue

  private final int m_runMotorID;
  private final WPI_TalonSRX m_runMotor;
  
  private final int ticksPerBall = 0;
  private final double arbitraryFeedForward = 0;
  
  private int m_powerCellCount;
  private boolean m_powerCellPreviouslyDetectedFront;
  private boolean m_powerCellPreviouslyDetectedBack;

  /**
   * Creates a new magazine 
   */
  public Magazine() {
    m_networkTable = NetworkTableInstance.getDefault().getTable(getName());
    m_directionReversedEntry = m_networkTable.getEntry("Direction reversed");
    m_powerCellCountEntry = m_networkTable.getEntry("Power cell count");

    // Properties defaultProperties = new Properties();
    properties = new Properties();
    try {
      InputStream deployStream = new FileInputStream(Filesystem.getDeployDirectory() + PROPERTIES_NAME);
      // InputStream robotStream = new FileInputStream(Filesystem.getOperatingDirectory() + PROPERTIES_NAME);
      // defaultProperties.load(deployStream);
      properties.load(deployStream);
      logger.info("succesfuly loaded");
    } catch(Exception e) {
      logger.log(Level.SEVERE, "failed to load", e);
    }

    m_runMotorID = Integer.parseInt("runMotorID");

    m_frontPhotoEyeChannel = Integer.parseInt(properties.getProperty("frontPhotoEyeChannel"));
    m_backPhotoEyeChannel = Integer.parseInt(properties.getProperty("backPhotoEyeChannel"));

    m_runMotor = new WPI_TalonSRX(m_runMotorID);

    // m_frontPhotoEye = new PhotoEye(m_frontPhotoEyeChannel);
    // m_backPhotoEye = new PhotoEye(m_backPhotoEyeChannel);
    m_frontPhotoEye = null;
    m_backPhotoEye = null;
  }

  @Override
  public void periodic() {
    // updatePowerCellCount();
    // m_powerCellPreviouslyDetectedFront = getPowerCellDetectedFront();
    // m_powerCellPreviouslyDetectedBack = getPowerCellDetectedBack();

    // m_directionReversedEntry.setBoolean(getDirectionReversed());
    // m_powerCellCountEntry.setNumber(getPowerCellCount());
  }

  public boolean getPowerCellDetectedFront() {
    if (m_frontPhotoEye.get()) logger.fine("power cell detected front");
    return m_frontPhotoEye.get();
  }

  public boolean getPowerCellDetectedBack() {
    if (m_backPhotoEye.get()) logger.fine("power cell detected back");
    return m_backPhotoEye.get();
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
  
  //might be temporary
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
    if (deltaCount != 0) logger.log(Level.FINER, "power cell count", m_powerCellCount);
  }

  public boolean getDirectionReversed() {
    return m_runMotor.getMotorOutputPercent() < 0;
  }
  
  public void setSpeed(double speed) {
    m_runMotor.set(ControlMode.PercentOutput, speed);
  }

  public void feedBalls(int amount) {
    m_runMotor.set(ControlMode.MotionMagic, amount * ticksPerBall, DemandType.ArbitraryFeedForward, arbitraryFeedForward);
  }
}
