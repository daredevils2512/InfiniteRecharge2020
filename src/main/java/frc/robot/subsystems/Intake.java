/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final int m_intake1ID = 31;
  private final WPI_TalonSRX m_intake1;

  private final int m_extenderForwardChannel = 2;
  private final int m_extenderReverseChannel = 3;
  private final DoubleSolenoid m_extender;
  private final DoubleSolenoid.Value m_extendedValue = Value.kForward;
  private final DoubleSolenoid.Value m_retractedValue = Value.kReverse;

  /**
   * Creates a new power cell intake.
   */
  public Intake() {
    m_intake1 = new WPI_TalonSRX(m_intake1ID);
    m_intake1.configFactoryDefault();

    m_extender = new DoubleSolenoid(m_extenderForwardChannel, m_extenderReverseChannel);
  }

  @Override
  public void periodic() {
    
  }

  public boolean getExtended() {
    return m_extender.get() == m_extendedValue;
  }

  public void setExtended(boolean wantsExtended) {
    m_extender.set(wantsExtended ? m_extendedValue : m_retractedValue);
  }

  public void run(double speed) {
    m_intake1.set(ControlMode.PercentOutput, speed);
  }
}
