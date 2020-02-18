/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.sensors.PhotoEye;

public class Magazine extends PropertySubsystem {
  private final NetworkTable m_networkTable;
  private final NetworkTableEntry m_directionReversedEntry;
  private final NetworkTableEntry m_powerCellCountEntry;

  private boolean m_frontPhotoEyeEnabled;
  private final int m_frontPhotoEyeChannel;
  private final PhotoEye m_frontPhotoEye; // Photo eye closest to the intake

  private final int m_runMotorID;
  private final WPI_TalonSRX m_runMotor;
  
  private final int ticksPerBall = 0;
  private final double arbitraryFeedForward = 0;

  private int m_powerCellCount;
  private boolean m_powerCellPreviouslyDetectedFront;

  private final Runnable m_incrementPowerCellCount;
  private final Runnable m_decrementPowerCellCount;

  /**
   * Creates a new magazine
   */
  public Magazine(Runnable incrementPowerCellCount, Runnable decrementPowerCellCount) {
    super(Magazine.class.getSimpleName());

    m_networkTable = NetworkTableInstance.getDefault().getTable(getName());
    m_directionReversedEntry = m_networkTable.getEntry("Direction reversed");
    m_powerCellCountEntry = m_networkTable.getEntry("Power cell count");

    m_runMotorID = Integer.parseInt(properties.getProperty("runMotorID"));
    m_frontPhotoEyeEnabled = Boolean.parseBoolean(properties.getProperty("frontPhotoEyeEnabled"));
    m_frontPhotoEyeChannel = Integer.parseInt(properties.getProperty("frontPhotoEyeChannel"));

    m_runMotor = new WPI_TalonSRX(m_runMotorID);
    m_runMotor.configFactoryDefault();
    m_runMotor.setInverted(InvertType.InvertMotorOutput);

    m_frontPhotoEye = m_frontPhotoEyeEnabled ? new PhotoEye(m_frontPhotoEyeChannel) : null;
    
    m_incrementPowerCellCount = incrementPowerCellCount;
    m_decrementPowerCellCount = decrementPowerCellCount;
  }

  @Override
  public void periodic() {
    updatePowerCellCount();
    m_powerCellPreviouslyDetectedFront = getPowerCellDetectedFront();
    
    m_directionReversedEntry.setBoolean(getDirectionReversed());
    m_powerCellCountEntry.setNumber(getPowerCellCount());
  }

  public boolean getPowerCellDetectedFront() {
    if (m_frontPhotoEyeEnabled) {
      if (m_frontPhotoEye.get())
        logger.fine("power cell detected front");
      return m_frontPhotoEye.get();
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
    // int deltaCount = 0;
    // if (!getPowerCellDetectedFront() && m_powerCellPreviouslyDetectedFront)
    //   deltaCount++;
    // if (getDirectionReversed())
    //   deltaCount = -deltaCount; // Counting direction is reversed if the magazine is being run backwards

    // int newCount = m_powerCellCount + deltaCount;
    // if (newCount < 0)
    //   logger.log(Level.WARNING, "Power cell count exceeded lower bounds");
    // else if (newCount > 3)
    //   logger.log(Level.WARNING, "Power cell count exceeded upper bounds");

    // m_powerCellCount = MathUtil.clamp(newCount, 0, 3);
    // if (deltaCount != 0)
    //   logger.log(Level.FINER, "power cell count", m_powerCellCount);

    if (!getPowerCellDetectedFront() && m_powerCellPreviouslyDetectedFront) {
      if (getDirectionReversed())
        m_decrementPowerCellCount.run();
      else
        m_incrementPowerCellCount.run();
    }
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
