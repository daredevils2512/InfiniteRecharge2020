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
import frc.robot.sensors.DummyDigitalInput;
import frc.robot.sensors.IDigitalInput;
import frc.robot.sensors.PhotoEye;
import frc.robot.subsystems.interfaces.IMagazine;

public class Magazine extends PropertySubsystem implements IMagazine {
  public static class MagazineMap {
    public int runMotorID = -1;
    public int photoEyeChannel = -1;
  }
  
  private final NetworkTable m_networkTable;
  private final NetworkTableEntry m_directionReversedEntry;
  
  private boolean m_photoEyeEnabled;
  private final IDigitalInput m_photoEye;

  private final WPI_TalonSRX m_runMotor;
  
  private final int ticksPerBall = 0;
  private final double arbitraryFeedForward = 0;

  private boolean m_powerCellPreviouslyDetected;

  private final Runnable m_incrementPowerCellCount;
  private final Runnable m_decrementPowerCellCount;

  /**
   * Creates a new magazine
   */
  public Magazine(MagazineMap magazineMap, Runnable incrementPowerCellCount, Runnable decrementPowerCellCount) {
    m_networkTable = NetworkTableInstance.getDefault().getTable(getName());
    m_directionReversedEntry = m_networkTable.getEntry("Direction reversed");

    m_runMotor = new WPI_TalonSRX(magazineMap.runMotorID);
    m_runMotor.setInverted(InvertType.InvertMotorOutput);

    m_photoEyeEnabled = Boolean.parseBoolean(m_properties.getProperty("photoEyeEnabled"));

    m_photoEye = m_photoEyeEnabled ? new PhotoEye(magazineMap.photoEyeChannel) : new DummyDigitalInput();

    m_incrementPowerCellCount = incrementPowerCellCount;
    m_decrementPowerCellCount = decrementPowerCellCount;
  }

  @Override
  public void periodic() {
    updatePowerCellCount();
    m_powerCellPreviouslyDetected = getPowerCellDetected();

    m_directionReversedEntry.setBoolean(getDirectionReversed());
  }

  @Override
  public boolean getPowerCellDetected() {
    if (m_photoEye.get())
      m_logger.fine("power cell detected");
    return m_photoEye.get();
  }

  // might be temporary
  @Override
  public void updatePowerCellCount() {
    if (!getPowerCellDetected() && m_powerCellPreviouslyDetected) {
      if (getDirectionReversed()) {
        m_decrementPowerCellCount.run();
      } else {
        m_incrementPowerCellCount.run();
      }
    }
  }

  @Override
  public boolean getDirectionReversed() {
    return m_runMotor.getMotorOutputPercent() < 0;
  }

  @Override
  public void setSpeed(double speed) {
    m_runMotor.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void feedBalls(int amount) {
    m_runMotor.set(ControlMode.MotionMagic, amount * ticksPerBall, DemandType.ArbitraryFeedForward,
        arbitraryFeedForward);
  }

  @Override
  public Map<String, Object> getValues() {
    return null;
  }
}
