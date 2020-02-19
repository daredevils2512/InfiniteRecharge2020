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
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.sensors.PhotoEye;

public class Queue extends PropertySubsystem {
  private final NetworkTable m_networkTable;
  private final NetworkTableEntry m_isClosedEntry;
  private final NetworkTableEntry m_runSpeedEntry;
  private final NetworkTableEntry m_hasPowerCellEntry;

  private final boolean m_photoEyeEnabled;
  private final int m_photoEyeChannel;
  private final PhotoEye m_photoEye;

  private final int m_runMotorID;
  private final TalonSRX m_runMotor;

  private final boolean m_gateEnabled;
  // TODO: Check all the gate wiring and stuff
  private final int m_gateForwardChannel;
  private final int m_gateReverseChannel;
  private final DoubleSolenoid.Value m_openValue = Value.kForward;
  private final DoubleSolenoid.Value m_closedValue = Value.kReverse;
  private final DoubleSolenoid m_gate;

  /**
   * Creates a new Queue.
   */
  public Queue() {
    super(Queue.class.getSimpleName());

    m_networkTable = NetworkTableInstance.getDefault().getTable(getName());
    m_runSpeedEntry = m_networkTable.getEntry("Run speed");
    m_isClosedEntry = m_networkTable.getEntry("Is closed");
    m_hasPowerCellEntry = m_networkTable.getEntry("Has power cell");

    m_photoEyeEnabled = Boolean.parseBoolean(properties.getProperty("photoEyeEnabled"));
    m_photoEyeChannel = Integer.parseInt(properties.getProperty("photoEyeChannel"));

    m_runMotorID = Integer.parseInt(properties.getProperty("runMotorID"));

    m_gateEnabled = Boolean.parseBoolean(properties.getProperty("gateEnabled"));
    m_gateForwardChannel = Integer.parseInt(properties.getProperty("gateForwardChannel"));
    m_gateReverseChannel = Integer.parseInt(properties.getProperty("gateReverseChannel"));

    m_photoEye = m_photoEyeEnabled ? new PhotoEye(m_photoEyeChannel) : null;

    m_runMotor = new TalonSRX(m_runMotorID);
    m_runMotor.configFactoryDefault();
    m_runMotor.setInverted(InvertType.InvertMotorOutput);
    m_runMotor.setNeutralMode(NeutralMode.Brake);

    m_gate = m_gateEnabled ? new DoubleSolenoid(m_gateForwardChannel, m_gateReverseChannel) : null;
  }

  @Override
  public void periodic() {
    m_runSpeedEntry.setNumber(m_runMotor.getMotorOutputPercent());
    m_isClosedEntry.setBoolean(getClosed());
    m_hasPowerCellEntry.setBoolean(hasPowerCell());
  }

  public void run(double speed) {
    m_runMotor.set(ControlMode.PercentOutput, speed);
  }

  public void run(double speed, boolean wantsClosed) {
    setClosed(wantsClosed);
    run(speed);
  }

  public boolean getClosed() {
    if (m_gateEnabled) {
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
    return m_photoEyeEnabled ? m_photoEye.get() : false;
  }

  @Override
  protected Map<String, Object> getValues() {
    return null;
  }
}
