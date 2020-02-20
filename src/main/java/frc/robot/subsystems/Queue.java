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
import frc.robot.sensors.PhotoEye;

public class Queue extends PropertySubsystem {
  public static class QueueMap {
    public int queueRunID = -1;
    public int photoEyeChannel = -1;
  }

  private boolean m_photoEyeEnabled;
  private final PhotoEye m_photoEye;

  public final NetworkTable m_networkTable;
  private final NetworkTableEntry m_runSpeedEntry;

  private final TalonSRX m_runMotor;

  /**
   * Creates a new Queue.
   */
  public Queue(QueueMap queueMap) {
    super(Queue.class);

    m_networkTable = NetworkTableInstance.getDefault().getTable(getName());
    m_runSpeedEntry = m_networkTable.getEntry("Run speed");

    m_runMotor = new TalonSRX(queueMap.queueRunID);
    m_runMotor.configFactoryDefault();
    m_runMotor.setInverted(InvertType.InvertMotorOutput);

    m_photoEye = m_photoEyeEnabled ? new PhotoEye(queueMap.photoEyeChannel) : null;
  }

  @Override
  public void periodic() {
    m_runSpeedEntry.setNumber(m_runMotor.getMotorOutputPercent());
  }

  public void run(double speed) {
    m_runMotor.set(ControlMode.PercentOutput, speed);
  }

  public boolean hasPowerCell() {
    if (m_photoEyeEnabled) {
      if (m_photoEye.get()) m_logger.fine("queue has power cell");
      return m_photoEye.get();
    } else {
      return false;
    }
  }

  @Override
  protected Map<String, Object> getValues() {
    return null;
  }
}
