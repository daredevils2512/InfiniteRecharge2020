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

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final NetworkTable m_networkTable;

  private final int m_extendMotorID = -1;
  private final int m_runMotorID = -1;
  private final TalonSRX m_extendMotor;
  private final TalonSRX m_runMotor;

  private final int m_extenderEncoderResolution = 4096; // TODO: Find intake extender encoder resolution
  private final double m_extenderGearRatio = 1; // TODO: Find intake extender gear ratio
  // TODO: Find the intake setpoint angles
  // Assume zero degrees is horizontal
  private final double m_retractedAngle = 90;
  private final double m_extendedAngle = 0;

  // TODO: Configure PID for intake extender
  private final int m_motionMagicSlot = 0;
  private final double m_pGain = 0;
  private final double m_iGain = 0;
  private final double m_dGain = 0;
  private final double m_arbitraryFeedForward = 0;

  private boolean m_extended = false;

  private boolean m_motionMagicEnabled = false;
  
  /**
   * Creates a new power cell intake
   */
  public Intake() {
    m_networkTable = NetworkTableInstance.getDefault().getTable(getName());

    m_extendMotor = new TalonSRX(m_extendMotorID);
    m_runMotor = new TalonSRX(m_runMotorID);
    m_extendMotor.configFactoryDefault();
    m_runMotor.configFactoryDefault();

    // Config PID for extender
    m_extendMotor.config_kP(m_motionMagicSlot, m_pGain);
    m_extendMotor.config_kI(m_motionMagicSlot, m_iGain);
    m_extendMotor.config_kD(m_motionMagicSlot, m_dGain);
  }

  @Override
  public void periodic() {
    // if(m_motionMagicEnabled) {
    //   double targetAngle = m_extended ? m_extendedAngle : m_retractedAngle;
    //   double targetPosition = toSensorUnits(targetAngle);
    //   double gravityScalar = Math.cos(Math.toRadians(targetAngle));
    //   m_extendMotor.set(ControlMode.MotionMagic, targetPosition, DemandType.ArbitraryFeedForward, m_arbitraryFeedForward * gravityScalar);
    // }

    m_networkTable.getEntry("Extended").setBoolean(m_extended);
  }

  public void setMotionMagicEnabled(boolean wantsEnabled) {
    m_motionMagicEnabled = wantsEnabled;
  }

  public boolean getExtended() {
    return m_extended;
  }

  public void setExtended(boolean wantsExtended) {
    m_extended = wantsExtended;
  }

  public void run(double speed) {
    m_runMotor.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Temporary function for testing/configuring the extender
   */
  public void runExtender(double speed) {
    m_extendMotor.set(ControlMode.PercentOutput, speed);
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
  private int toEncoderTicks(double degrees) {
    return (int)(degrees / 360 / m_extenderGearRatio * m_extenderEncoderResolution);
  }
}
