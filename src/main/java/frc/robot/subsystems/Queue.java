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
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.sensors.DummyDigitalInput;
import frc.robot.sensors.IDigitalInput;
import frc.robot.sensors.PhotoEye;

public class Queue extends PropertySubsystem {
  private final int m_photoEyeChannel;
  private final IDigitalInput m_photoEye;

  public final NetworkTable m_networkTable;
  public final NetworkTableEntry m_isClosedEntry;
  private final NetworkTableEntry m_runSpeedEntry;

  private final int m_runMotorID;
  private final TalonSRX m_runMotor;

  private final boolean m_gateEnabled;
  // TODO: Check all the gate wiring and stuff
  private final int m_gateForwardChannel;
  private final int m_gateReverseChannel;
  private final DoubleSolenoid.Value m_openValue = Value.kForward;
  private final DoubleSolenoid.Value m_closedValue = Value.kReverse;
  private final DoubleSolenoid m_gate;

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

    m_gateEnabled = Boolean.parseBoolean(properties.getProperty("gateEnabled"));
    m_gateForwardChannel = Integer.parseInt(properties.getProperty("gateForwardChannel"));
    m_gateReverseChannel = Integer.parseInt(properties.getProperty("gateReverseChannel"));
    m_gate = m_gateEnabled ? new DoubleSolenoid(m_gateForwardChannel, m_gateReverseChannel) : null;

    m_incrementMagazinePowerCellCount = incrementMagazinePowerCellCount;
    m_decrementMagazinePowerCellCount = decrementMagazinePowerCellCount;
  }

  @Override
  public void periodic() {
    updateMagazinePowerCellCount();
    m_runSpeedEntry.setNumber(m_runMotor.getMotorOutputPercent());
    m_isClosedEntry.setBoolean(getClosed());
  }

  private void updateMagazinePowerCellCount() {
    if (m_photoEye.get() && !m_powerCellPreviouslyDetected && !getDirectionReversed()) {
      m_decrementMagazinePowerCellCount.run();
    } else if (!m_photoEye.get() && m_powerCellPreviouslyDetected && getDirectionReversed()) {
      m_incrementMagazinePowerCellCount.run();
    }
  }

  public void run(double speed) {
    m_runMotor.set(ControlMode.PercentOutput, speed);
  }

  public void run(double speed, boolean wantsClosed) {
    setClosed(wantsClosed);
    run(speed);
  }

  public boolean getDirectionReversed() {
    return m_runMotor.getMotorOutputPercent() < 0;
  }

  public boolean getClosed() {
    if (m_gateEnabled) {
      if (m_gate.get() == m_closedValue) logger.fine("gate closed"); 
      return m_gate.get() == m_closedValue;
    } else {
      return false;
    }
  }

  public void setClosed(boolean wantsClosed) {
    if (m_gateEnabled) {
      m_gate.set(wantsClosed ? m_closedValue : m_openValue);
    }
  }

  public boolean hasPowerCell() {
      if (m_photoEye.get()) logger.fine("queue has power cell");
      return m_photoEye.get();
  }

  @Override
  protected Map<String, Object> getValues() {
    return null;
  }
}
