/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.Properties;
import java.util.logging.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private static Logger logger = Logger.getLogger(Shooter.class.getName());
  private final NetworkTable m_networkTable;
  private final Properties properties;
  private static final String PROPERTIES_NAME = "/shooter.properties";

  private final int m_shooterID;
  private final int m_hoodID;
  private final TalonSRX m_shooter;
  private final TalonSRX m_hood;

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
    Properties defaultProperties = new Properties();
    properties = new Properties(defaultProperties);
    try {
      InputStream deployStream = new FileInputStream(Filesystem.getDeployDirectory() + PROPERTIES_NAME);
      InputStream robotStream = new FileInputStream(Filesystem.getOperatingDirectory() + PROPERTIES_NAME);
      defaultProperties.load(deployStream);
      properties.load(robotStream);
      logger.info("succesfuly loaded");
    } catch(IOException e) {
      logger.log(Level.SEVERE, "failed to save", e);
    }

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

    m_shooter = new TalonSRX(m_shooterID);
    m_hood = new TalonSRX(m_hoodID);

    m_shooter.configFactoryDefault();
    m_shooter.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    m_shooter.config_kP(m_shooterVelocityPIDSlot, m_shooterVelocityPGain);
    m_shooter.config_kI(m_shooterVelocityPIDSlot, m_shooterVelocityIGain);
    m_shooter.config_kD(m_shooterVelocityPIDSlot, m_shooterVelocityDGain);

    m_hood.configFactoryDefault();
    m_hood.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

    m_hood.config_kP(m_hoodPositionPIDSlot, m_hoodPositionPGain);
    m_hood.config_kI(m_hoodPositionPIDSlot, m_hoodPositionIGain);
    m_hood.config_kD(m_hoodPositionPIDSlot, m_hoodPositionDGain);
    
    m_shooter.setNeutralMode(NeutralMode.Coast); // Drains less battery >true
    
    m_hood.setNeutralMode(NeutralMode.Brake);
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
    m_hood.config_kP(m_hoodPositionPIDSlot, m_hoodPositionPGain);
    m_hood.config_kI(m_hoodPositionPIDSlot, m_hoodPositionIGain);
    m_hood.config_kD(m_hoodPositionPIDSlot, m_hoodPositionDGain);

    m_networkTable.getEntry("Velocity (RPM)").setDouble(getVelocity());
    m_networkTable.getEntry("Percent output").setDouble(m_shooter.getMotorOutputPercent());
    m_networkTable.getEntry("Shooter velocity P gain").setDouble(m_shooterVelocityPGain);
    m_networkTable.getEntry("Shooter velocity I gain").setDouble(m_shooterVelocityIGain);
    m_networkTable.getEntry("Shooter velocity D gain").setDouble(m_shooterVelocityDGain);
  }

  public void resetHoodAngle(double angle) {
    m_hood.setSelectedSensorPosition(toEncoderPulsesHood(angle));
  }

  public void setPercentOutput(double speed) {
    m_shooter.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Set closed loop velocity control target
   * @param targetVelocity Target velocity in revolutions per minute
   */
  public void setTargetVelocity(double velocity) {
    m_shooter.selectProfileSlot(m_shooterVelocityPIDSlot, 0);
    m_shooter.set(ControlMode.Velocity, toEncoderPulsesPer100Milliseconds(velocity));
  }

  public void stop() {
    m_shooter.set(ControlMode.PercentOutput, 0);
  }

  /**
   * Set hood target angle for position PID
   * @param angle Angle in degrees
   */
  public void setTargetAngle(double angle) {
    m_hood.set(ControlMode.Position, toEncoderPulsesHood(angle));
  }

  /**
   * Get shooter velocity
   * @return Velocity in revolutions per minute
   */
  public double getVelocity() {
    return toRPM(m_shooter.getSelectedSensorVelocity());
  }

  /**
   * Get hood angle
   * @return Angle in degrees
   */
  public double getAngle() {
    return toAngleHood(m_hood.getSelectedSensorPosition());
  }

  private int toEncoderPulsesPer100Milliseconds(double rpm) {
    // To encoder pulses then to 100ms
    return (int)(rpm / m_shooterGearRatio * m_shooterEncoderResolution / 60 / 10);
  }

  private double toRPM(int encoderPulsesPer100Milliseconds) {
    return (double)(encoderPulsesPer100Milliseconds * 10 * 60) / m_shooterEncoderResolution / m_shooterGearRatio;
  }

  private int toEncoderPulsesHood(double angle) {
    return (int)(angle / 360 / m_hoodGearRatio * m_hoodEncoderResolution);
  }

  private double toAngleHood(int encoderPulses) {
    return (double)encoderPulses / m_hoodEncoderResolution * m_hoodGearRatio * 360;
  }

  public void savePID() {
    try {
      OutputStream outputStream = new FileOutputStream(Filesystem.getOperatingDirectory() + PROPERTIES_NAME);
      properties.setProperty("shooterVelocityPGain", "" + m_shooterVelocityPGain);
      properties.setProperty("shooterVelocityIGain", "" + m_shooterVelocityIGain);
      properties.setProperty("shooterVelocityDGain", "" + m_shooterVelocityDGain);

      properties.setProperty("hoodPositionPGain", "" + m_hoodPositionPGain);
      properties.setProperty("hoodPositionIGain", "" + m_hoodPositionIGain);
      properties.setProperty("hoodPositionDGain", "" + m_hoodPositionDGain);
      properties.store(outputStream, "saved PId or somethbings");
      logger.info("succesfuly saved");
    } catch(IOException e) {
      logger.log(Level.SEVERE, "failed to save", e);
    }
  }
}
