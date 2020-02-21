/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;
import java.util.logging.Level;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.interfaces.IShooter;

public class Shooter extends PropertySubsystem implements IShooter {
  public static class ShooterMap {
    public int shooter1ID = -1;
    public int shooter2ID = -1;
    public int shooterHoodID = -1;
  }

  private final NetworkTable m_networkTable;
  private final NetworkTableEntry m_shooterOutputEntry;
  private final NetworkTableEntry m_shooterVelocityEntry;
  private final NetworkTableEntry m_shooterPGainEntry;
  private final NetworkTableEntry m_shooterIGainEntry;
  private final NetworkTableEntry m_shooterDGainEntry;
  private final NetworkTableEntry m_hoodPGainEntry;
  private final NetworkTableEntry m_hoodIGainEntry;
  private final NetworkTableEntry m_hoodDGainEntry;

  private final TalonSRX m_shooter;
  private final TalonSRX m_shooterFollower;
  private final TalonSRX m_hood;
  private final Boolean m_hoodEnabled;

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
  public Shooter(ShooterMap shooterMap) {
    m_networkTable = NetworkTableInstance.getDefault().getTable(getName());
    m_shooterOutputEntry = m_networkTable.getEntry("Shooter output");
    m_shooterVelocityEntry = m_networkTable.getEntry("Shooter RPM");
    m_shooterPGainEntry = m_networkTable.getEntry("Shooter P gain");
    m_shooterIGainEntry = m_networkTable.getEntry("Shooter I gain");
    m_shooterDGainEntry = m_networkTable.getEntry("Shooter D gain");
    m_hoodPGainEntry = m_networkTable.getEntry("Hood P gain");
    m_hoodIGainEntry = m_networkTable.getEntry("Hood I gain");
    m_hoodDGainEntry = m_networkTable.getEntry("Hood D gain");

    m_hoodEnabled = Boolean.parseBoolean(m_properties.getProperty("hoodEnabled"));
    m_shooterEncoderResolution = Integer.parseInt(m_properties.getProperty("shooterEncoderResolution"));
    m_hoodEncoderResolution = Integer.parseInt(m_properties.getProperty("hoodEncoderResolution"));
    m_shooterGearRatio = Double.parseDouble(m_properties.getProperty("shooterGearRatio"));
    m_hoodGearRatio = Double.parseDouble(m_properties.getProperty("hoodGearRatio"));

    m_shooterVelocityPIDSlot = Integer.parseInt(m_properties.getProperty("shooterVelocityPIDSlot"));
    m_shooterVelocityPGain = Double.parseDouble(m_properties.getProperty("shooterVelocityPGain"));
    m_shooterVelocityIGain = Double.parseDouble(m_properties.getProperty("shooterVelocityIGain"));
    m_shooterVelocityDGain = Double.parseDouble(m_properties.getProperty("shooterVelocityDGain"));

    m_hoodPositionPIDSlot = Integer.parseInt(m_properties.getProperty("hoodPositionPIDSlot"));
    m_hoodPositionPGain = Double.parseDouble(m_properties.getProperty("hoodPositionPGain"));
    m_hoodPositionIGain = Double.parseDouble(m_properties.getProperty("hoodPositionIGain"));
    m_hoodPositionDGain = Double.parseDouble(m_properties.getProperty("hoodPositionDGain"));

    m_shooter = new TalonSRX(shooterMap.shooter1ID);
    m_shooterFollower = new TalonSRX(shooterMap.shooter2ID);
    m_shooter.configFactoryDefault();
    m_shooterFollower.configFactoryDefault();

    m_shooterFollower.follow(m_shooter);

    m_shooter.setInverted(InvertType.None);
    m_shooterFollower.setInverted(InvertType.FollowMaster);

    m_shooter.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    m_shooter.config_kP(m_shooterVelocityPIDSlot, m_shooterVelocityPGain);
    m_shooter.config_kI(m_shooterVelocityPIDSlot, m_shooterVelocityIGain);
    m_shooter.config_kD(m_shooterVelocityPIDSlot, m_shooterVelocityDGain);

    if (m_hoodEnabled) {
      m_hood = new TalonSRX(shooterMap.shooterHoodID);
      m_hood.configFactoryDefault();
      m_hood.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
      m_hood.setNeutralMode(NeutralMode.Brake);
    
      m_hood.config_kP(m_hoodPositionPIDSlot, m_hoodPositionPGain);
      m_hood.config_kI(m_hoodPositionPIDSlot, m_hoodPositionIGain);
      m_hood.config_kD(m_hoodPositionPIDSlot, m_hoodPositionDGain);
    } else {
      m_hood = null;
    }
  }

  @Override
  public void periodic() {
    // Remove once PID is tuned
    m_shooterVelocityPGain = m_shooterPGainEntry.getDouble(m_shooterVelocityPGain);
    m_shooterVelocityIGain = m_shooterIGainEntry.getDouble(m_shooterVelocityIGain);
    m_shooterVelocityDGain = m_shooterDGainEntry.getDouble(m_shooterVelocityDGain);
    m_hoodPositionPGain = m_hoodPGainEntry.getDouble(m_hoodPositionPGain);
    m_hoodPositionIGain = m_hoodIGainEntry.getDouble(m_hoodPositionIGain);
    m_hoodPositionDGain = m_hoodDGainEntry.getDouble(m_hoodPositionDGain);

    m_shooter.config_kP(m_shooterVelocityPIDSlot, m_shooterVelocityPGain);
    m_shooter.config_kI(m_shooterVelocityPIDSlot, m_shooterVelocityIGain);
    m_shooter.config_kD(m_shooterVelocityPIDSlot, m_shooterVelocityDGain);

    if (m_hoodEnabled) {
      m_hood.config_kP(m_hoodPositionPIDSlot, m_hoodPositionPGain);
      m_hood.config_kI(m_hoodPositionPIDSlot, m_hoodPositionIGain);
      m_hood.config_kD(m_hoodPositionPIDSlot, m_hoodPositionDGain);
    }

    m_shooterVelocityEntry.setDouble(getVelocity());
    m_shooterOutputEntry.setDouble(m_shooter.getMotorOutputPercent());
    m_shooterPGainEntry.setDouble(m_shooterVelocityPGain);
    m_shooterIGainEntry.setDouble(m_shooterVelocityIGain);
    m_shooterDGainEntry.setDouble(m_shooterVelocityDGain);
  }

  @Override
  public void resetHoodAngle(double angle) {
    if (m_hoodEnabled) m_hood.setSelectedSensorPosition(toEncoderPulsesHood(angle));
  }

  @Override
  public void setPercentOutput(double speed) {
    m_shooter.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Set closed loop velocity control target
   * 
   * @param targetVelocity Target velocity in revolutions per minute
   */
  @Override
  public void setTargetVelocity(double velocity) {
    m_logger.log(Level.FINER, "setting velocity to = ", velocity);
    m_shooter.selectProfileSlot(m_shooterVelocityPIDSlot, 0);
    m_shooter.set(ControlMode.Velocity,
    toEncoderPulsesPer100Milliseconds(velocity));
  }

  @Override
  public void stop() {
    m_shooter.set(ControlMode.PercentOutput, 0);
  }

  /**
   * Set hood target angle for position PID
   * 
   * @param angle Angle in degrees
   */
  @Override
  public void setTargetAngle(double angle) {
    if (m_hoodEnabled) m_hood.set(ControlMode.Position, toEncoderPulsesHood(angle));
  }

  /**
   * Get shooter velocity
   * 
   * @return Velocity in revolutions per minute
   */
  @Override
  public double getVelocity() {
    return toRPM(m_shooter.getSelectedSensorVelocity());
  }

  /**
   * Get hood angle
   * 
   * @return Angle in degrees
   */
  @Override
  public double getAngle() {
    return m_hoodEnabled ? toAngleHood(m_hood.getSelectedSensorPosition()) : 0.0;
  }

  private int toEncoderPulsesPer100Milliseconds(double rpm) {
    // To encoder pulses then to 100ms
    return (int) (rpm / m_shooterGearRatio * m_shooterEncoderResolution / 60 / 10);
  }

  private double toRPM(int encoderPulsesPer100Milliseconds) {
    return (double) (encoderPulsesPer100Milliseconds * 10 * 60) / m_shooterEncoderResolution / m_shooterGearRatio;
  }

  private int toEncoderPulsesHood(double angle) {
    return (int) (angle / 360 / m_hoodGearRatio * m_hoodEncoderResolution);
  }

  private double toAngleHood(int encoderPulses) {
    return (double) encoderPulses / m_hoodEncoderResolution * m_hoodGearRatio * 360;
  }

  @Override
  public Map<String, Object> getValues() {
    Map<String, Object> values = new HashMap<>();
    values.put("shooterVelocityPGain", m_shooterVelocityPGain);
    values.put("shooterVelocityIGain", m_shooterVelocityIGain);
    values.put("shooterVelocityDGain", m_shooterVelocityDGain);
    if (m_hoodEnabled) {
      values.put("hoodPositionPGain", m_hoodPositionPGain);
      values.put("hoodPositionIGain", m_hoodPositionIGain);
      values.put("hoodPositionDGain", m_hoodPositionDGain);
    }
    return values;
  }
}
