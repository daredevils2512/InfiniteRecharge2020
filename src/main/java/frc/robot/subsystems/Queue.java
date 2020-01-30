/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Queue extends SubsystemBase {
  public final NetworkTable m_networkTable;
  public final NetworkTableEntry m_isClosedEntry;
  private final NetworkTableEntry m_runSpeedEntry;

  private final int m_runMotorID = -1;
  private final TalonSRX m_runMotor;

  // TODO: Check all the gate wiring and stuff
  private final int m_gateForwardChannel = -1;
  private final int m_gateReverseChannel = -1;
  private final DoubleSolenoid.Value m_openValue = Value.kForward;
  private final DoubleSolenoid.Value m_closedValue = Value.kReverse;
  private final DoubleSolenoid m_gate;

  private final int m_photoeyeID = -1;
  private final DigitalInput m_photoeye;

  /**
   * Creates a new Queue.
   */
  public Queue() {
    m_networkTable = NetworkTableInstance.getDefault().getTable(getName());
    m_runSpeedEntry = m_networkTable.getEntry("Run speed");
    m_isClosedEntry = m_networkTable.getEntry("Is closed");

    m_runMotor = new TalonSRX(m_runMotorID);
    m_runMotor.configFactoryDefault();

    m_gate = new DoubleSolenoid(m_gateForwardChannel, m_gateReverseChannel);

    m_photoeye = new DigitalInput(m_photoeyeID);
  }

  @Override
  public void periodic() {
    m_runSpeedEntry.setNumber(m_runMotor.getMotorOutputPercent());
    m_isClosedEntry.setBoolean(getIsClosed());
  }

  public void run(double speed) {
    m_runMotor.set(ControlMode.PercentOutput, speed);
  }

  public boolean getIsClosed() {
    return m_gate.get() == m_closedValue;
  }

  public void setClosed(boolean wantsClosed) {
    m_gate.set(wantsClosed ? m_closedValue : m_openValue);
  }

  public boolean getBall() {
    return m_photoeye.get();
  }
}
