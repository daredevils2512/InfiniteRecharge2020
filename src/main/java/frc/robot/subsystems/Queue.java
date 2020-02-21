/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.sensors.DummyDigitalInput;
import frc.robot.sensors.IDigitalInput;
import frc.robot.sensors.PhotoEye;
import frc.robot.subsystems.interfaces.IQueue;

public class Queue extends PropertySubsystem implements IQueue {
  public static class QueueMap {
    public int queueRunID = -1;
    public int photoEyeChannel = -1;
  }

  private boolean m_photoEyeEnabled;
  private final IDigitalInput m_photoEye;

  public final NetworkTable m_networkTable;
  private final NetworkTableEntry m_runSpeedEntry;

  private final TalonSRX m_runMotor;

  private final Runnable m_incrementMagazinePowerCellCount;
  private final Runnable m_decrementMagazinePowerCellCount;

  private boolean m_powerCellPreviouslyDetected = false;

  /**
   * Creates a new Queue.
   */
  public Queue(QueueMap queueMap, Runnable incrementMagazinePowerCellCount, Runnable decrementMagazinePowerCellCount) {
    m_networkTable = NetworkTableInstance.getDefault().getTable(getName());
    m_runSpeedEntry = m_networkTable.getEntry("Run speed");

    m_runMotor = new TalonSRX(queueMap.queueRunID);
    m_runMotor.configFactoryDefault();
    m_runMotor.setInverted(InvertType.InvertMotorOutput);

    m_photoEye = m_photoEyeEnabled ? new PhotoEye(queueMap.photoEyeChannel) : new DummyDigitalInput();

    m_incrementMagazinePowerCellCount = incrementMagazinePowerCellCount;
    m_decrementMagazinePowerCellCount = decrementMagazinePowerCellCount;
  }

  @Override
  public void periodic() {
    updateMagazinePowerCellCount();

    m_runSpeedEntry.setNumber(m_runMotor.getMotorOutputPercent());
  }

  private void updateMagazinePowerCellCount() {
    if (m_photoEye.get() && !m_powerCellPreviouslyDetected) {
      m_powerCellPreviouslyDetected = true;
    } else if (!m_photoEye.get() && m_powerCellPreviouslyDetected) {
      if (!getDirectionReversed()) {
        m_decrementMagazinePowerCellCount.run();
      } else {
        m_incrementMagazinePowerCellCount.run();
      }
      m_powerCellPreviouslyDetected = false;
    }
  }

  @Override
  public void run(double speed) {
    m_runMotor.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public boolean getDirectionReversed() {
    return m_runMotor.getMotorOutputPercent() < 0;
  }

  @Override
  public boolean hasPowerCell() {
    if (m_photoEye.get()) m_logger.fine("queue has power cell");
    return m_photoEye.get();
  }

  @Override
  public Map<String, Object> getValues() {
    return null;
  }
}
