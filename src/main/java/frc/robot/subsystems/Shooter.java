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
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.interfaces.IShooter;

public class Shooter extends PropertySubsystem implements IShooter {
  public static class ShooterMap {
    public int shooter1ID = -1;
    public int shooter2ID = -1;
    public int shooterHoodID = -1;
  }

  private final NetworkTable m_networkTable;

  private final TalonSRX m_shooter;
  private final TalonSRX m_shooterFollower;
  private final TalonSRX m_hood;

  private final int m_shooterEncoderResolution;
  private final int m_hoodEncoderResolution;
  private final double m_shooterGearRatio;
  private final double m_hoodGearRatio;

  // TODO: Tune shooter velocity PID
  private final int m_shooterVelocityPIDSlot;
  private double m_shooterVelocityPGain = 0;
  private double m_shooterVelocityIGain = 0;
  private double m_shooterVelocityDGain = 0;

  // TODO: Tun shooter hood position PID
  private final int m_hoodPositionPIDSlot;
  private double m_hoodPositionPGain = 0;
  private double m_hoodPositionIGain = 0;
  private double m_hoodPositionDGain = 0;

  /**
   * Creates a new power cell shooter
   */
  public Shooter(ShooterMap shooterMap) {
    m_networkTable = NetworkTableInstance.getDefault().getTable(getName());

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

    m_shooter.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    m_shooter.config_kP(m_shooterVelocityPIDSlot, m_shooterVelocityPGain);
    m_shooter.config_kI(m_shooterVelocityPIDSlot, m_shooterVelocityIGain);
    m_shooter.config_kD(m_shooterVelocityPIDSlot, m_shooterVelocityDGain);

    m_shooter.setInverted(InvertType.None);
    m_shooterFollower.setInverted(InvertType.FollowMaster);

    m_shooter.setNeutralMode(NeutralMode.Coast);
    m_shooter.setSensorPhase(false);

    m_hood = new TalonSRX(shooterMap.shooterHoodID);
    m_hood.configFactoryDefault();
    m_hood.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    m_hood.config_kP(m_hoodPositionPIDSlot, m_hoodPositionPGain);
    m_hood.config_kI(m_hoodPositionPIDSlot, m_hoodPositionIGain);
    m_hood.config_kD(m_hoodPositionPIDSlot, m_hoodPositionDGain);

    m_hood.setInverted(InvertType.None);
    m_hood.setNeutralMode(NeutralMode.Brake);
    m_hood.setSensorPhase(false);
  }

  @Override
  public void periodic() {
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

    m_networkTable.getEntry("Velocity (RPM)").setDouble(getShooterVelocity());
    m_networkTable.getEntry("Percent output").setDouble(m_shooter.getMotorOutputPercent());
    m_networkTable.getEntry("Shooter velocity P gain").setDouble(m_shooterVelocityPGain);
    m_networkTable.getEntry("Shooter velocity I gain").setDouble(m_shooterVelocityIGain);
    m_networkTable.getEntry("Shooter velocity D gain").setDouble(m_shooterVelocityDGain);
  }

  @Override
  public void resetHoodAngle(double angle) {
    m_hood.setSelectedSensorPosition(toHoodEncoderPulses(angle));
  }

  @Override
  public void setShooterPercentOutput(double speed) {
    m_shooter.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Set closed loop velocity control target
   * 
   * @param targetVelocity Target velocity in revolutions per minute
   */
  @Override
  public void setShooterVelocity(double velocity) {
    m_shooter.selectProfileSlot(m_shooterVelocityPIDSlot, 0);
    m_shooter.set(ControlMode.Velocity, toShooterEncoderPulsesPer100Milliseconds(velocity));
  }

  @Override
  public boolean shooterAtSetpoint(double tolerance) {
    double velocityError = toShooterRPM(m_shooter.getClosedLoopError(m_shooterVelocityPIDSlot));
    return Math.abs(velocityError) < tolerance;
  }

  /**
   * Set hood target angle for position PID
   * 
   * @param angle Angle in degrees
   */
  public void setHoodAngle(double angle) {
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

  @Override
  public boolean hoodAtSetpoint(double tolerance) {
    double angleError = toHoodAngle(m_hood.getClosedLoopError(m_hoodPositionPIDSlot));
    return Math.abs(angleError) < tolerance;
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
  public Map<String, Object> getValues() {
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
