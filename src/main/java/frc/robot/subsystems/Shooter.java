/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final int m_velocitySlot = 0;
  // TODO: Tune shooter PIDF
  private final double m_kP = 0;
  private final double m_kI = 0;
  private final double m_kD = 0;
  private final double m_kF = 0;

  private final int m_encoderResolution = 4096; // TODO: Check shooter PPR

  // TODO: Config shooter CAN
  private final int m_shooterID = 21;
  private final WPI_TalonSRX m_shooter;

  /**
   * Creates a new power cell shooter.
   */
  public Shooter() {
    m_shooter = new WPI_TalonSRX(m_shooterID);

    m_shooter.configFactoryDefault();
    m_shooter.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    m_shooter.config_kP(m_velocitySlot, m_kP);
    m_shooter.config_kI(m_velocitySlot, m_kI);
    m_shooter.config_kD(m_velocitySlot, m_kD);
    m_shooter.config_kF(m_velocitySlot, m_kF);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter velocity", getVelocity());
  }

  public void percentOutput(double speed) {
    m_shooter.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Set closed loop velocity control target
   * @param targetVelocity Target velocity in RPM
   */
  public void setTargetVelocity(double targetVelocityRPM) {
    int targetVelocityPulsesPer100MS = toPulsesPer100MS(targetVelocityRPM); // Convert to proper units
    m_shooter.set(ControlMode.Velocity, targetVelocityPulsesPer100MS);
  }

  /**
   * Get shooter velocity
   * @return Velocity in RPM
   */
  public double getVelocity() {
    return toRPM(m_shooter.getSelectedSensorPosition());
  }

  private int toPulsesPer100MS(double rpm) {
    return (int)(rpm * m_encoderResolution / 60 / 10);
  }

  private double toRPM(int pulsesPer100MS) {
    return (double)(pulsesPer100MS * 10 * 60) / m_encoderResolution;
  }
}
