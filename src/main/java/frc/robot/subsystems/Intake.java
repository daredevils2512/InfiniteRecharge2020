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

      // TODO: Measure intake extender 
      intakeExtenderConfig.motionCruiseVelocity = 0;
      intakeExtenderConfig.motionAcceleration = 0;
      intakeExtenderConfig.motionCurveStrength = 0;
    }
  }

  private final IntakeConfig m_intakeConfig;
  private final int m_extenderID = -1;
  private final int m_topIntakeID = -1;
  private final int m_bottomIntakeID = -1;
  private final TalonSRX m_extender;
  private final TalonSRX m_topIntake;
  private final TalonSRX m_bottomIntake;

  private final int m_extenderEncoderResolution = 4096; // TODO: Find intake extender encoder res
  private final double m_extenderGearRatio = 1; // TODO: Find intake extender gear ratio
  // TODO: Find the intake setpoint angles
  // Assume zero degrees is horizontal
  private final double m_retractedAngle = 90;
  private final double m_extendedAngle = 0;

  // TODO: Configure PID for intake extender
  private final int m_pidSlot = 0;
  private final double m_pGain = 0;
  private final double m_iGain = 0;
  private final double m_dGain = 0;
  private final double m_arbitraryFeedForward = 0;

  private boolean m_extended = false;

  /**
   * Creates a new power cell intake
   */
  public Intake() {
    m_intakeConfig = new IntakeConfig();
    m_extender = new TalonSRX(m_extenderID);
    m_topIntake = new TalonSRX(m_topIntakeID);
    m_bottomIntake = new TalonSRX(m_bottomIntakeID);
    m_extender.configAllSettings(m_intakeConfig.intakeExtenderConfig);
    m_topIntake.configAllSettings(m_intakeConfig.intakeTopConfig);
    m_bottomIntake.configAllSettings(m_intakeConfig.intakeBottomConfig);

    // Config PID for extender
    m_extender.config_kP(m_pidSlot, m_pGain);
    m_extender.config_kI(m_pidSlot, m_iGain);
    m_extender.config_kD(m_pidSlot, m_dGain);
  }

  @Override
  public void periodic() {
    double targetAngle = m_extended ? m_extendedAngle : m_retractedAngle;
    double targetPosition = toSensorUnits(targetAngle);
    double gravityScalar = Math.cos(Math.toRadians(targetAngle));
    m_extender.set(ControlMode.MotionMagic, targetPosition, DemandType.ArbitraryFeedForward, m_arbitraryFeedForward * gravityScalar);
  }

  public boolean getExtended() {
    return m_extended;
  }

  public void setExtended(boolean wantsExtended) {
    m_extended = wantsExtended;
  }

  public void run(double speed) {
    m_topIntake.set(ControlMode.PercentOutput, speed);
    m_bottomIntake.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Temporary function for testing/configuring the extender
   */
  public void runExtender_Temp(double speed) {
    int position = m_extender.getSelectedSensorPosition();
    double angle = toDegrees(position);
    if (speed < 0 && angle <= m_retractedAngle) {
      speed = 0;
    } else if (speed > 0 && angle >= m_extendedAngle) {
      speed = 0;
    }
    m_extender.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Convert from raw sensor units to an angle in degrees
   * <p>Applies only the the extender
   * @param sensorUnits
   * @return Angle in degrees
   */
  private double toDegrees(int sensorUnits) {
    return (double)sensorUnits / m_extenderEncoderResolution * m_extenderGearRatio * 360;
  }

  /**
   * Convert from an angle in degrees to raw sensor units
   * <p>Applies only to the extender
   * @param degrees
   * @return
   */
  private int toSensorUnits(double degrees) {
    return (int)(degrees / 360 / m_extenderGearRatio * m_extenderEncoderResolution);
  }
}
