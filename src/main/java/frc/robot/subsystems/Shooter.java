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
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.subsystems.interfaces.IShooter;

public class Shooter extends PropertySubsystem implements IShooter {
  public static class ShooterMap {
    public int shooter1ID = -1;
    public int shooter2ID = -1;
    public int shooterHoodID = -1;
  }

  //{@Link https://www.desmos.com/calculator/hlfz61fwpw}

  /**
   * notes
   * 
   * front of trench: status: good 
   * distance: 4.8
   * set rpm: 6k
   * hood pos: 1900
   * 
   * 6.2 : 6500
   * 5.7 : 6000
   * 2.9 : 6000
   * 4.0 : 5500
   * 4.3 : 5700
   * 
   */

  private final NetworkTable m_networkTable;
  private final NetworkTableEntry m_shooterOutputEntry;
  private final NetworkTableEntry m_hoodPositionEntry;
  private final NetworkTableEntry m_shooterVelocityEntry;
  private final NetworkTableEntry m_shooterPGainEntry;
  private final NetworkTableEntry m_shooterIGainEntry;
  private final NetworkTableEntry m_shooterDGainEntry;
  private final NetworkTableEntry m_shooterFGainEntry;
  private final NetworkTableEntry m_arbitraryFeedForwardEntry;
  private final NetworkTableEntry m_hoodPGainEntry;
  private final NetworkTableEntry m_hoodIGainEntry;
  private final NetworkTableEntry m_hoodDGainEntry;
  private final NetworkTableEntry m_setShooterSpeed;
  private final NetworkTableEntry m_shooterSpeedSetter;
  private final NetworkTableEntry m_targetVelocity;

  private final TalonSRX m_shooter;
  private final TalonSRX m_shooterFollower;
  private final TalonSRX m_hood;
  private final Boolean m_hoodEnabled;
  private final double m_hoodForwardSoftLimit;
  private final double m_hoodReverseSoftLimit;

  private final int m_shooterEncoderResolution;
  private final int m_hoodEncoderResolution;
  private final double m_shooterGearRatio; // TODO: Check shooter gearing
  private final double m_hoodGearRatio; // TODO: Check shooter hood gearing
  private final double m_hoodRadius;
  private final double m_hoodStartingPosition;
  private final double m_hoodCircumference;
  private final double m_hoodMMPerTooth;

  private final int m_shooterVelocityPIDSlot;
  // TODO: Tune shooter velocity PID
  // Change to final once configured
  private double m_shooterVelocityPGain = 0;
  private double m_shooterVelocityIGain = 0;
  private double m_shooterVelocityDGain = 0;
  private double m_shooterVelocityFGain = 0;
  private double m_arbitraryFeedForward = 0;

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
    m_hoodPositionEntry = m_networkTable.getEntry("hood position");
    m_shooterPGainEntry = m_networkTable.getEntry("Shooter P gain");
    m_shooterIGainEntry = m_networkTable.getEntry("Shooter I gain");
    m_shooterDGainEntry = m_networkTable.getEntry("Shooter D gain");
    m_shooterFGainEntry = m_networkTable.getEntry("Shooter F gain");
    m_arbitraryFeedForwardEntry = m_networkTable.getEntry("arbitrary feed forward");
    m_hoodPGainEntry = m_networkTable.getEntry("Hood P gain");
    m_hoodIGainEntry = m_networkTable.getEntry("Hood I gain");
    m_hoodDGainEntry = m_networkTable.getEntry("Hood D gain");
    m_setShooterSpeed = m_networkTable.getEntry("set shooter RPM");
    m_shooterSpeedSetter = m_networkTable.getEntry("set shooter speed toggle");
    m_targetVelocity = m_networkTable.getEntry("shooter target velocity");
    m_shooterSpeedSetter.setDouble(0.0);

    m_hoodEnabled = Boolean.parseBoolean(m_properties.getProperty("hoodEnabled"));
    m_shooterEncoderResolution = Integer.parseInt(m_properties.getProperty("shooterEncoderResolution"));
    m_hoodEncoderResolution = Integer.parseInt(m_properties.getProperty("hoodEncoderResolution"));
    m_shooterGearRatio = Double.parseDouble(m_properties.getProperty("shooterGearRatio"));
    m_hoodRadius = Units.inchesToMeters(Double.parseDouble(m_properties.getProperty("hoodRadius")));
    m_hoodStartingPosition = Double.parseDouble(m_properties.getProperty("hoodStartingPosition")); //should be degrees
    m_hoodMMPerTooth = Double.parseDouble(m_properties.getProperty("hoodMMPerTooth")) / 1000;

    m_hoodForwardSoftLimit = Double.parseDouble(m_properties.getProperty("hoodForwardSoftLimit"));
    m_hoodReverseSoftLimit = Double.parseDouble(m_properties.getProperty("hoodReverseSoftLimit"));

    m_shooterVelocityPIDSlot = Integer.parseInt(m_properties.getProperty("shooterVelocityPIDSlot"));
    m_shooterVelocityPGain = Double.parseDouble(m_properties.getProperty("shooterVelocityPGain"));
    m_shooterVelocityIGain = Double.parseDouble(m_properties.getProperty("shooterVelocityIGain"));
    m_shooterVelocityDGain = Double.parseDouble(m_properties.getProperty("shooterVelocityDGain"));
    m_shooterVelocityFGain = Double.parseDouble(m_properties.getProperty("shooterVelocityFGain"));
    m_arbitraryFeedForward = Double.parseDouble(m_properties.getProperty("arbitraryFeedForward"));

    m_hoodPositionPIDSlot = Integer.parseInt(m_properties.getProperty("hoodPositionPIDSlot"));
    m_hoodPositionPGain = Double.parseDouble(m_properties.getProperty("hoodPositionPGain"));
    m_hoodPositionIGain = Double.parseDouble(m_properties.getProperty("hoodPositionIGain"));
    m_hoodPositionDGain = Double.parseDouble(m_properties.getProperty("hoodPositionDGain"));

    m_hoodCircumference = m_hoodRadius * 2 * Math.PI;
    m_hoodGearRatio = 1 / m_hoodCircumference * m_hoodMMPerTooth;

    m_shooter = new TalonSRX(shooterMap.shooter1ID);
    m_shooterFollower = new TalonSRX(shooterMap.shooter2ID);
    m_shooter.configFactoryDefault();
    m_shooterFollower.configFactoryDefault();

    m_shooterFollower.follow(m_shooter);

    m_shooter.setInverted(InvertType.InvertMotorOutput);
    m_shooterFollower.setInverted(InvertType.None);

    m_shooter.setNeutralMode(NeutralMode.Coast);
    m_shooterFollower.setNeutralMode(NeutralMode.Coast);

    m_shooter.configContinuousCurrentLimit(30);
    m_shooter.configPeakCurrentDuration(0);
    m_shooterFollower.configContinuousCurrentLimit(30);
    m_shooterFollower.configPeakCurrentDuration(0);

    m_shooter.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    m_shooter.config_kP(m_shooterVelocityPIDSlot, m_shooterVelocityPGain);
    m_shooter.config_kI(m_shooterVelocityPIDSlot, m_shooterVelocityIGain);
    m_shooter.config_kD(m_shooterVelocityPIDSlot, m_shooterVelocityDGain);
    m_shooter.config_kF(m_shooterVelocityPIDSlot, m_shooterVelocityFGain);

    if (m_hoodEnabled) {
      m_hood = new TalonSRX(shooterMap.shooterHoodID);
      m_hood.setSelectedSensorPosition(0);
      m_hood.configFactoryDefault();
      m_hood.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
      m_hood.setNeutralMode(NeutralMode.Brake);
      m_hood.setInverted(InvertType.InvertMotorOutput);
    
      m_hood.config_kP(m_hoodPositionPIDSlot, m_hoodPositionPGain);
      m_hood.config_kI(m_hoodPositionPIDSlot, m_hoodPositionIGain);
      m_hood.config_kD(m_hoodPositionPIDSlot, m_hoodPositionDGain);

      m_hood.configForwardSoftLimitThreshold(2000); //hardcoded for now
      m_hood.configReverseSoftLimitThreshold(0);

      m_hood.configForwardSoftLimitEnable(true);
      m_hood.configReverseSoftLimitEnable(true);
   } else {
      m_hood = null;
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("master shooter current output ", m_shooter.getStatorCurrent());
    SmartDashboard.putNumber("master shooter current supply ", m_shooter.getSupplyCurrent());
    m_hoodPositionEntry.setDouble(m_hood.getSelectedSensorPosition());

    // Remove once PID is tuned
    m_shooterVelocityPGain = m_shooterPGainEntry.getDouble(m_shooterVelocityPGain);
    m_shooterVelocityIGain = m_shooterIGainEntry.getDouble(m_shooterVelocityIGain);
    m_shooterVelocityDGain = m_shooterDGainEntry.getDouble(m_shooterVelocityDGain);
    m_shooterVelocityFGain = m_shooterFGainEntry.getDouble(m_shooterVelocityFGain);
    m_arbitraryFeedForward = m_arbitraryFeedForwardEntry.getDouble(m_arbitraryFeedForward);
    m_hoodPositionPGain = m_hoodPGainEntry.getDouble(m_hoodPositionPGain);
    m_hoodPositionIGain = m_hoodIGainEntry.getDouble(m_hoodPositionIGain);
    m_hoodPositionDGain = m_hoodDGainEntry.getDouble(m_hoodPositionDGain);

    m_shooter.config_kP(m_shooterVelocityPIDSlot, m_shooterVelocityPGain);
    m_shooter.config_kI(m_shooterVelocityPIDSlot, m_shooterVelocityIGain);
    m_shooter.config_kD(m_shooterVelocityPIDSlot, m_shooterVelocityDGain);
    m_shooter.config_kF(m_shooterVelocityPIDSlot, m_shooterVelocityFGain);

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
    m_shooterFGainEntry.setDouble(m_shooterVelocityFGain);
    if (m_shooterSpeedSetter.getBoolean(false)) {
      setTargetVelocity(m_setShooterSpeed.getDouble(0));
    }
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
    m_targetVelocity.setDouble(velocity);
    m_shooter.selectProfileSlot(m_shooterVelocityPIDSlot, 0);
    m_shooter.set(ControlMode.Velocity, toEncoderPulsesPer100Milliseconds(velocity), DemandType.ArbitraryFeedForward, m_arbitraryFeedForward);
    m_shooter.set(ControlMode.Velocity,
      toEncoderPulsesPer100Milliseconds(velocity));
  }

  @Override
  public void stop() {
    m_shooter.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void setHoodSpeed(double speed) {
    if (m_hoodEnabled) {
      m_hood.set(ControlMode.PercentOutput, speed);
    }
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
    return (double) encoderPulses / m_hoodEncoderResolution * m_hoodGearRatio * 360 + m_hoodStartingPosition;
  }

  @Override
  public Map<String, Object> getValues() {
    Map<String, Object> values = new HashMap<>();
    values.put("shooterVelocityPGain", m_shooterVelocityPGain);
    values.put("shooterVelocityIGain", m_shooterVelocityIGain);
    values.put("shooterVelocityDGain", m_shooterVelocityDGain);
    values.put("shooterVelocityFGain", m_shooterVelocityFGain);
    values.put("arbitraryFeedForward", m_arbitraryFeedForward);
    if (m_hoodEnabled) {
      values.put("hoodPositionPGain", m_hoodPositionPGain);
      values.put("hoodPositionIGain", m_hoodPositionIGain);
      values.put("hoodPositionDGain", m_hoodPositionDGain);
    }
    return values;
  }
}