/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final NetworkTable m_networkTable;

  private final int m_shooterID = -1;
  private final int m_hoodID = -1;
  private final TalonSRX m_shooter;
  private final TalonSRX m_hood;

  private final int m_encoderResolution = 4096; // TODO: Check shooter encoder resolution
  private final double m_shooterGearRatio = 1;
  private final double m_hoodGearRatio = 1;

  private final int m_shooterVelocityPID = 0;
  // TODO: Tune shooter velocity PID
  // Change to final once configured
  private double m_shooterVelocityPGain = 0;
  private double m_shooterVelocityIGain = 0;
  private double m_shooterVelocityDGain = 0;

  private final int m_hoodPositionPID = 0;
  private double m_hoodPositionPGain = 0;
  private double m_hoodPositionIGain = 0;
  private double m_hoodPositionDGain = 0;

  /**
   * Creates a new power cell shooter
   */
  public Shooter() {
    m_networkTable = NetworkTableInstance.getDefault().getTable(getName());

    m_shooter = new TalonSRX(m_shooterID);
    m_hood = new TalonSRX(m_hoodID);

    m_shooter.configFactoryDefault();
    m_shooter.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    m_shooter.config_kP(m_shooterVelocityPID, m_shooterVelocityPGain);
    m_shooter.config_kI(m_shooterVelocityPID, m_shooterVelocityIGain);
    m_shooter.config_kD(m_shooterVelocityPID, m_shooterVelocityDGain);

    m_hood.configFactoryDefault();
    m_hood.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

    m_hood.config_kP(m_hoodPositionPID, m_hoodPositionPGain);
    m_hood.config_kI(m_hoodPositionPID, m_hoodPositionIGain);
    m_hood.config_kD(m_hoodPositionPID, m_hoodPositionDGain);
    
    m_shooter.setNeutralMode(NeutralMode.Coast); // Drains less battery
    
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

    m_shooter.config_kP(m_shooterVelocityPID, m_shooterVelocityPGain);
    m_shooter.config_kI(m_shooterVelocityPID, m_shooterVelocityIGain);
    m_shooter.config_kD(m_shooterVelocityPID, m_shooterVelocityDGain);
    m_hood.config_kP(m_hoodPositionPID, m_hoodPositionPGain);
    m_hood.config_kI(m_hoodPositionPID, m_hoodPositionIGain);
    m_hood.config_kD(m_hoodPositionPID, m_hoodPositionDGain);

    m_networkTable.getEntry("Velocity (RPM)").setDouble(getVelocity());
    m_networkTable.getEntry("Percent output").setDouble(m_shooter.getMotorOutputPercent());
    m_networkTable.getEntry("Shooter velocity P gain").setDouble(m_shooterVelocityPGain);
    m_networkTable.getEntry("Shooter velocity I gain").setDouble(m_shooterVelocityIGain);
    m_networkTable.getEntry("Shooter velocity D gain").setDouble(m_shooterVelocityDGain);
  }

  public void resetHoodAngle(double angle) {
    int encoderPosition = (int)(angle / 360 / m_hoodGearRatio * m_encoderResolution);
    m_hood.setSelectedSensorPosition(encoderPosition);
  }

  public void setPercentOutput(double speed) {
    m_shooter.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Set closed loop velocity control target
   * @param targetVelocity Target velocity in revolutions per minute
   */
  public void setTargetVelocity(double velocity) {
    int sensorUnitsPer100MS = toSensorUnitsPer100MS(velocity); // Convert to proper units for velocity control
    m_shooter.set(ControlMode.Velocity, sensorUnitsPer100MS);
  }

  public void stop() {
    m_shooter.set(ControlMode.PercentOutput, 0);
  }

  /**
   * Set hood target angle for position PID
   * @param angle Angle in degrees
   */
  public void setTargetAngle(double angle) {
    int encoderPosition = (int)(angle / 360 * m_hoodGearRatio * m_encoderResolution);
    m_hood.set(ControlMode.Position, encoderPosition);
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
    return (double)m_hood.getSelectedSensorPosition() / m_encoderResolution / m_hoodGearRatio * 360;
  }

  private int toSensorUnitsPer100MS(double rpm) {
    return (int)(rpm * m_shooterGearRatio * m_encoderResolution / 60 / 10);
  }

  private double toRPM(int sensorUnitsPer100MS) {
    return (double)(sensorUnitsPer100MS * 10 * 60) / m_encoderResolution / m_shooterGearRatio;
  }
}
