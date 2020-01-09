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

public class Turret extends SubsystemBase {
  private final NetworkTable m_networkTable;

  private final int m_turretMasterID = -1; // TODO: Configure CAN on turret
  private final TalonSRX m_turretMaster;

  // TODO: Find encoder and gearing details for turret
  private final double m_encoderResolution = -1;
  private final double m_gearRatio = -1;

  // TODO: Tune position PID
  private final int m_positionSlot = 0;
  private final double m_P = 0;
  private final double m_I = 0;
  private final double m_D = 0;

  /**
   * Creates a new turret
   */
  public Turret() {
    m_networkTable = NetworkTableInstance.getDefault().getTable(getName());

    m_turretMaster = new TalonSRX(m_turretMasterID);
    m_turretMaster.configFactoryDefault();

    m_turretMaster.config_IntegralZone(m_positionSlot, 0);
    m_turretMaster.config_kD(m_positionSlot, m_D);
    m_turretMaster.config_kI(m_positionSlot, m_I);
    m_turretMaster.config_kP(m_positionSlot, m_P);
    m_turretMaster.configClosedLoopPeakOutput(m_positionSlot, 1.0);
    m_turretMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

    m_turretMaster.setNeutralMode(NeutralMode.Brake);
    m_turretMaster.set(ControlMode.PercentOutput, 0);
    m_turretMaster.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    m_networkTable.getEntry("Encoder position").setNumber(getPosition());
    m_networkTable.getEntry("P gain").setNumber(m_P);
    m_networkTable.getEntry("I gain").setNumber(m_I);
    m_networkTable.getEntry("D gain").setNumber(m_D);
  }

  private int getPosition() {
    return m_turretMaster.getSelectedSensorPosition();
  }

  /**
   * Get the current angle of the turret
   * @return Angle in radians
   */
  public double getAngle() {
    // Convert from encoder pulses to radians
    return toRadians(getPosition());
  }

  public void resetEncoder() {
    m_turretMaster.setSelectedSensorPosition(0);
  }

  public void setSpeed(double speed) {
    m_turretMaster.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Set a target angle for position PID
   * @param angle Angle in radians
   */
  public void setTargetAngle(double angle) {
    m_turretMaster.set(ControlMode.Position, toEncoderPulses(angle));
  }

  private double toRadians(int encoderPulses) {
    return (double)encoderPulses / m_encoderResolution * m_gearRatio * 2 * Math.PI;
  }

  private double toEncoderPulses(double radians) {
    return radians / Math.PI / 2 / m_gearRatio * m_encoderResolution;
  }
}
