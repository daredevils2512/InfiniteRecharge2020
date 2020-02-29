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
import frc.robot.utils.Trigger;

public class Queue extends PropertySubsystem implements IQueue {
  public static class QueueMap {
    public int queueRunID = -1;
    public int photoEyeChannel = -1;
  }

  private boolean m_photoEyeEnabled;
  private final IDigitalInput m_photoEye;
  private final Trigger m_photoEyeTrigger;

  public final NetworkTable m_networkTable;
  private final NetworkTableEntry m_runSpeedEntry;

  private final TalonSRX m_runMotor;

  private Runnable m_onPowerCellInMagazine = () -> { };
  private Runnable m_onPowerCellInShooter = () -> { };
  private Runnable m_onPowerCellOutMagazine = () -> { };
  private Runnable m_onPowerCellOutShooter = () -> { };

  /**
   * Creates a new Queue.
   */
  public Queue(QueueMap queueMap) {
    m_networkTable = NetworkTableInstance.getDefault().getTable(getName());
    m_runSpeedEntry = m_networkTable.getEntry("Run speed");

    m_runMotor = new TalonSRX(queueMap.queueRunID);
    m_runMotor.configFactoryDefault();
    m_runMotor.setInverted(InvertType.InvertMotorOutput);

    m_photoEye = m_photoEyeEnabled ? new PhotoEye(queueMap.photoEyeChannel) : new DummyDigitalInput();
    m_photoEyeTrigger = new Trigger(() -> m_photoEye.get());
    m_photoEyeTrigger.whenActive(() -> {
      if (getDirectionReversed()) {
        m_onPowerCellInShooter.run();
      } else {
        m_onPowerCellInMagazine.run();
      }
    });
    m_photoEyeTrigger.whenInactive(() -> {
      if (getDirectionReversed()) {
        m_onPowerCellOutMagazine.run();
      } else {
        m_onPowerCellOutShooter.run();
      }
    });
  }

  @Override
  public void periodic() {
    m_photoEyeTrigger.update();

    m_runSpeedEntry.setNumber(m_runMotor.getMotorOutputPercent());
  }

  public void onPowerCellInMagazine(Runnable runnable) {
    m_onPowerCellInMagazine = runnable;
  }

  public void onPowerCellInShooter(Runnable runnable) {
    m_onPowerCellInShooter = runnable;
  }

  public void onPowerCellOutMagazine(Runnable runnable) {
    m_onPowerCellOutMagazine = runnable;
  }

  public void onPowerCellOutShooter(Runnable runnable) {
    m_onPowerCellOutShooter = runnable;
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
