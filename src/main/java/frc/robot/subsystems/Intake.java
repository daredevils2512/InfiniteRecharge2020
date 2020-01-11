/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final class IntakeConfig {
    public final TalonSRXConfiguration intakeExtenderConfig;
    public final TalonSRXConfiguration intakeTopConfig;
    public final TalonSRXConfiguration intakeBottomConfig;

    public IntakeConfig() {
      intakeExtenderConfig = new TalonSRXConfiguration();
      intakeTopConfig = new TalonSRXConfiguration();
      intakeBottomConfig = new TalonSRXConfiguration();

      intakeExtenderConfig.motionCruiseVelocity = 0;
      intakeExtenderConfig.motionAcceleration = 0;
      intakeExtenderConfig.motionCurveStrength = 0;
    }
  }

  private final IntakeConfig m_intakeConfig;
  private final int m_extenderID = -1;
  private final int m_intakeTopID = -1;
  private final int m_intakeBottomID = -1;
  private final TalonSRX m_extender;
  private final TalonSRX m_intakeTop;
  private final TalonSRX m_intakeBottom;

  private final int m_extenderEncoderRes = 4096; // TODO: Find intake extender encoder res
  private final double m_extenderGearRatio = 1; // TODO: Find intake extender gear ratio

  // PID for intake extender
  private final int m_motionMagicSlot = 0;
  private final double m_pGain = 0;
  private final double m_iGain = 0;
  private final double m_dGain = 0;
  private double m_arbitraryFeedForward = 0;

  private final int m_extendedPosition = 0; // TODO: Find the intake encoder position for fully extended

  private boolean m_extended = false;

  /**
   * Creates a new power cell intake
   */
  public Intake() {
    m_intakeConfig = new IntakeConfig();
    m_extender = new TalonSRX(m_extenderID);
    m_intakeTop = new TalonSRX(m_intakeTopID);
    m_intakeBottom = new TalonSRX(m_intakeBottomID);
    m_extender.configAllSettings(m_intakeConfig.intakeExtenderConfig);
    m_intakeTop.configAllSettings(m_intakeConfig.intakeTopConfig);
    m_intakeBottom.configAllSettings(m_intakeConfig.intakeBottomConfig);

    // Config PID for intake extender
    m_extender.config_kP(m_motionMagicSlot, m_pGain);
    m_extender.config_kI(m_motionMagicSlot, m_iGain);
    m_extender.config_kD(m_motionMagicSlot, m_dGain);
  }

  @Override
  public void periodic() {
    double targetPosition = m_extended ? m_extendedPosition : 0;

    m_extender.set(ControlMode.MotionMagic, targetPosition, DemandType.ArbitraryFeedForward, m_arbitraryFeedForward);
  }

  public boolean getExtended() {
    return m_extended;
  }

  public void setExtended(boolean wantsExtended) {
    int targetPosition = wantsExtended ? m_extendedPosition : 0;
    // TODO: Configure intake extender PID
    // m_intakeExtender.set(ControlMode.Position, targetPosition);
    m_extender.set(ControlMode.MotionMagic, targetPosition, DemandType.ArbitraryFeedForward, m_arbitraryFeedForward);
    m_extended = wantsExtended;
  }

  public void run(double speed) {
    m_intakeTop.set(ControlMode.PercentOutput, speed);
    m_intakeBottom.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Temporary function for testing/configuring the extender
   */
  public void runExtender_Temp(double speed) {
    double pos = m_extender.getSelectedSensorPosition();
    if (speed < 0 && pos <= 0) {
      speed = 0;
    } else if (speed > 0 && pos >= m_extendedPosition) {
      speed = 0;
    }
    m_extender.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Convert from raw sensor units on the intake extender to an angle in degrees
   * @param encoderPulses Raw sensor units of intake extender
   * @return Angle in degrees
   */
  private double toDegrees(int sensorUnits) {
    return (double)sensorUnits / m_extenderEncoderRes * m_extenderGearRatio;
  }
}
