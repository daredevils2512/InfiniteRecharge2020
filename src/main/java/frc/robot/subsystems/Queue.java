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

  /**
   * Creates a new Queue.
   */
  public Queue(QueueMap queueMap) {
    m_networkTable = NetworkTableInstance.getDefault().getTable(getName());
    m_runSpeedEntry = m_networkTable.getEntry("Run speed");
    m_photoEyeEnabled = Boolean.parseBoolean(m_properties.getProperty("photoEyeEnabled"));

    m_runMotor = new TalonSRX(queueMap.queueRunID);
    m_runMotor.configFactoryDefault();
    m_runMotor.setInverted(InvertType.InvertMotorOutput);

    m_photoEye = m_photoEyeEnabled ? new PhotoEye(queueMap.photoEyeChannel) : new DummyDigitalInput();
  }

  @Override
  public void periodic() {

    m_runSpeedEntry.setNumber(m_runMotor.getMotorOutputPercent());
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

  @Override
  public IDigitalInput getPhotoEye() {
    return this.m_photoEye;
  }
}
