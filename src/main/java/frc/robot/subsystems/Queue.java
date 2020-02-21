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
  private final int m_photoEyeChannel;
  private final IDigitalInput m_photoEye;

  public final NetworkTable m_networkTable;
  public final NetworkTableEntry m_isClosedEntry;
  private final NetworkTableEntry m_runSpeedEntry;

  private final int m_runMotorID;
  private final TalonSRX m_runMotor;

  private final Runnable m_incrementMagazinePowerCellCount;
  private final Runnable m_decrementMagazinePowerCellCount;

  private boolean m_powerCellPreviouslyDetected = false;

  /**
   * Creates a new Queue.
   */
  public Queue(Runnable incrementMagazinePowerCellCount, Runnable decrementMagazinePowerCellCount) {
    super(Queue.class.getName());

    m_networkTable = NetworkTableInstance.getDefault().getTable(getName());
    m_runSpeedEntry = m_networkTable.getEntry("Run speed");
    m_isClosedEntry = m_networkTable.getEntry("Is closed");

    m_runMotorID = Integer.parseInt(properties.getProperty("runMotorID"));
    m_photoEyeChannel = Integer.parseInt(properties.getProperty("photoEyeChannel"));

    m_runMotor = new TalonSRX(m_runMotorID);
    m_runMotor.configFactoryDefault();
    m_runMotor.setInverted(InvertType.InvertMotorOutput);

    if (Boolean.parseBoolean(properties.getProperty("photoEyeEnabled")))
      m_photoEye = new PhotoEye(m_photoEyeChannel);
    else
      m_photoEye = new DummyDigitalInput();

    m_incrementMagazinePowerCellCount = incrementMagazinePowerCellCount;
    m_decrementMagazinePowerCellCount = decrementMagazinePowerCellCount;
  }

  @Override
  public void periodic() {
    updateMagazinePowerCellCount();
    m_runSpeedEntry.setNumber(m_runMotor.getMotorOutputPercent());
  }

  private void updateMagazinePowerCellCount() {
    if (m_photoEye.get() && !m_powerCellPreviouslyDetected && !getDirectionReversed()) {
      m_decrementMagazinePowerCellCount.run();
    } else if (!m_photoEye.get() && m_powerCellPreviouslyDetected && getDirectionReversed()) {
      m_incrementMagazinePowerCellCount.run();
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
      if (m_photoEye.get()) logger.fine("queue has power cell");
      return m_photoEye.get();
  }

  @Override
  public Map<String, Object> getValues() {
    return null;
  }
}
