/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Shooter extends PropertySubsystem {
  private final NetworkTable m_networkTable;

  private final boolean m_hoodEnabled;

  private final int m_shooterID;
  private final int m_hoodID;
  private final WPI_TalonFX m_shooter;
  private final WPI_TalonFX m_hood;

  private final int m_shooterEncoderResolution;
  private final int m_hoodEncoderResolution; // TODO: Check shooter encoder resolution
  private final double m_shooterGearRatio; // TODO: Check shooter gearing
  private final double m_hoodGearRatio; // TODO: Check shooter hood gearing

  private final int m_shooterVelocityPIDSlot;
  // TODO: Tune shooter velocity PID
  // Change to final once configured
  private double m_shooterVelocityPGain = 0;
  private double m_shooterVelocityIGain = 0;
  private double m_shooterVelocityDGain = 0;

  private final int m_hoodPositionPIDSlot;
  private double m_hoodPositionPGain = 0;
  private double m_hoodPositionIGain = 0;
  private double m_hoodPositionDGain = 0;

  /**
   * Creates a new power cell shooter
   */
  public Shooter() {
    super(Shooter.class.getSimpleName());

    m_hoodEnabled = Boolean.parseBoolean(properties.getProperty("hoodEnabled"));

    m_shooterID = Integer.parseInt(properties.getProperty("shooterID"));
    m_hoodID = Integer.parseInt(properties.getProperty("hoodID"));

    m_shooterEncoderResolution = Integer.parseInt(properties.getProperty("shooterEncoderResolution"));
    m_hoodEncoderResolution = Integer.parseInt(properties.getProperty("hoodEncoderResolution"));
    m_shooterGearRatio = Double.parseDouble(properties.getProperty("shooterGearRatio"));
    m_hoodGearRatio = Double.parseDouble(properties.getProperty("hoodGearRatio"));

    m_shooterVelocityPIDSlot = Integer.parseInt(properties.getProperty("shooterVelocityPIDSlot"));
    m_shooterVelocityPGain = Double.parseDouble(properties.getProperty("shooterVelocityPGain"));
    m_shooterVelocityIGain = Double.parseDouble(properties.getProperty("shooterVelocityIGain"));
    m_shooterVelocityDGain = Double.parseDouble(properties.getProperty("shooterVelocityDGain"));

    m_hoodPositionPIDSlot = Integer.parseInt(properties.getProperty("hoodPositionPIDSlot"));
    m_hoodPositionPGain = Double.parseDouble(properties.getProperty("hoodPositionPGain"));
    m_hoodPositionIGain = Double.parseDouble(properties.getProperty("hoodPositionIGain"));
    m_hoodPositionDGain = Double.parseDouble(properties.getProperty("hoodPositionDGain"));

    m_networkTable = NetworkTableInstance.getDefault().getTable(getName());

    m_shooter = new WPI_TalonFX(m_shooterID);
    m_shooter.configFactoryDefault();
    // Sensor phase should be automatically adjusted for Falcon 500
    m_shooter.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    m_shooter.config_kP(m_shooterVelocityPIDSlot, m_shooterVelocityPGain);
    m_shooter.config_kI(m_shooterVelocityPIDSlot, m_shooterVelocityIGain);
    m_shooter.config_kD(m_shooterVelocityPIDSlot, m_shooterVelocityDGain);

    m_shooter.setInverted(InvertType.None);
    m_shooter.setNeutralMode(NeutralMode.Coast);

    if (m_hoodEnabled) {
      m_hood = new WPI_TalonFX(m_hoodID);
      m_hood.configFactoryDefault();
      m_hood.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

      m_hood.config_kP(m_hoodPositionPIDSlot, m_hoodPositionPGain);
      m_hood.config_kI(m_hoodPositionPIDSlot, m_hoodPositionIGain);
      m_hood.config_kD(m_hoodPositionPIDSlot, m_hoodPositionDGain);

      m_hood.setInverted(InvertType.None);
      m_hood.setNeutralMode(NeutralMode.Brake);
      m_hood.setSensorPhase(false);
    } else {
      m_hood = null;
    }
  }

  @Override
  public void periodic() {
    // Remove once PID is tuned
    m_shooterVelocityPGain = m_networkTable.getEntry("Shooter velocity P gain").getDouble(m_shooterVelocityPGain);
    m_shooterVelocityIGain = m_networkTable.getEntry("Shooter velocity I gain").getDouble(m_shooterVelocityIGain);
    m_shooterVelocityDGain = m_networkTable.getEntry("Shooter velocity D gain").getDouble(m_shooterVelocityDGain);
    m_hoodPositionPGain = m_networkTable.getEntry("Hood position P gain").getDouble(m_shooterVelocityPGain);
    m_hoodPositionIGain = m_networkTable.getEntry("Hood position I gain").getDouble(m_shooterVelocityIGain);
    m_hoodPositionDGain = m_networkTable.getEntry("Hood position D gain").getDouble(m_shooterVelocityDGain);

    m_shooter.config_kP(m_shooterVelocityPIDSlot, m_shooterVelocityPGain);
    m_shooter.config_kI(m_shooterVelocityPIDSlot, m_shooterVelocityIGain);
    m_shooter.config_kD(m_shooterVelocityPIDSlot, m_shooterVelocityDGain);

    if (m_hoodEnabled) {
      m_hood.config_kP(m_hoodPositionPIDSlot, m_hoodPositionPGain);
      m_hood.config_kI(m_hoodPositionPIDSlot, m_hoodPositionIGain);
      m_hood.config_kD(m_hoodPositionPIDSlot, m_hoodPositionDGain);
    }

    m_networkTable.getEntry("Velocity (RPM)").setDouble(getShooterVelocity());
    m_networkTable.getEntry("Percent output").setDouble(m_shooter.getMotorOutputPercent());
    m_networkTable.getEntry("Shooter velocity P gain").setDouble(m_shooterVelocityPGain);
    m_networkTable.getEntry("Shooter velocity I gain").setDouble(m_shooterVelocityIGain);
    m_networkTable.getEntry("Shooter velocity D gain").setDouble(m_shooterVelocityDGain);
  }

  public void resetHoodAngle(double angle) {
    if (m_hoodEnabled)
      m_hood.setSelectedSensorPosition(toHoodEncoderPulses(angle));
  }

  public void setShooterPercentOutput(double speed) {
    m_shooter.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Set closed loop velocity control target
   * 
   * @param targetVelocity Target velocity in revolutions per minute
   */
  public void setTargetVelocity(double velocity) {
    m_shooter.selectProfileSlot(m_shooterVelocityPIDSlot, 0);
    m_shooter.set(ControlMode.Velocity, toShooterEncoderPulsesPer100Milliseconds(velocity));
  }

  /**
   * Set hood target angle for position PID
   * 
   * @param angle Angle in degrees
   */
  public void setHoodAngle(double angle) {
    if (m_hoodEnabled)
      m_hood.set(ControlMode.Position, toHoodEncoderPulses(angle));
  }

  /**
   * Get shooter velocity
   * 
   * @return Velocity in revolutions per minute
   */
  public double getShooterVelocity() {
    return toShooterRPM(m_shooter.getSelectedSensorVelocity());
  }

  /**
   * Get hood angle
   * 
   * @return Angle in degrees
   */
  public double getHoodAngle() {
    return toHoodAngle(m_hood.getSelectedSensorPosition());
  }

  private int toShooterEncoderPulsesPer100Milliseconds(double rpm) {
    // To encoder pulses then to 100ms
    return (int)(rpm / m_shooterGearRatio * m_shooterEncoderResolution / 60 / 10);
  }

  private double toShooterRPM(int encoderPulsesPer100Milliseconds) {
    // To minutes then to revolutions
    return (double)(encoderPulsesPer100Milliseconds * 10 * 60) / m_shooterEncoderResolution / m_shooterGearRatio;
  }

  private int toHoodEncoderPulses(double angle) {
    return (int)(angle / 360 / m_hoodGearRatio * m_hoodEncoderResolution);
  }

  private double toHoodAngle(int encoderPulses) {
    return (double)encoderPulses / m_hoodEncoderResolution * m_hoodGearRatio * 360;
  }

  @Override
  protected Map<String, Object> getValues() {
    Map<String, Object> values = new HashMap<>();
    values.put("shooterVelocityPGain", m_shooterVelocityPGain);
    values.put("shooterVelocityIGain", m_shooterVelocityIGain);
    values.put("shooterVelocityDGain", m_shooterVelocityDGain);

    values.put("hoodPositionPGain", m_hoodPositionPGain);
    values.put("hoodPositionIGain", m_hoodPositionIGain);
    values.put("hoodPositionDGain", m_hoodPositionDGain);

    return values;
  }
}
