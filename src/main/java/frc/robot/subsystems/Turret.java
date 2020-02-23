/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;
import java.util.logging.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.interfaces.ITurret;
import frc.robot.utils.DareMathUtil;

public class Turret extends PropertySubsystem implements ITurret {
  public static class TurretMap {
    public int turretID = -1;
  }

  private final NetworkTable m_networkTable;
  private final NetworkTableEntry m_angleEntry;
  private final NetworkTableEntry m_wrappedAngleEntry;

  private final TalonSRX m_turretMaster;

  // TODO: Find encoder and gearing details for turret
  private final double m_encoderResolution;
  private final double m_gearRatio;
  private final double m_maxTurnDegrees;
  private final double m_tolerance; //in degrees

  private final double m_minAngle; // Angle in degrees
  private final double m_maxAngle; // Angle in degrees

  // TODO: Tune position PID
  private final int m_positionSlot;
  private double m_P = 0;
  private double m_I = 0;
  private double m_D = 0;
  private int m_motionAcceleration;
  private int m_motionCruiseVelocity;

  /**
   * Creates a new turret
   */
  public Turret(TurretMap turretMap) {
    m_encoderResolution = Integer.parseInt(m_properties.getProperty("encoderResolution"));
    m_gearRatio = Double.parseDouble(m_properties.getProperty("gearRatio"));
    m_maxTurnDegrees = Double.parseDouble(m_properties.getProperty("maxTurnDegrees"));
    m_tolerance = Double.parseDouble(m_properties.getProperty("tolerance"));
    m_maxAngle = Double.parseDouble(m_properties.getProperty("maxAngle"));
    m_minAngle = Double.parseDouble(m_properties.getProperty("minAngle"));

    m_positionSlot = Integer.parseInt(m_properties.getProperty("positionSlot"));
    m_P = Double.parseDouble(m_properties.getProperty("P"));
    m_I = Double.parseDouble(m_properties.getProperty("I"));
    m_D = Double.parseDouble(m_properties.getProperty("D"));

    m_networkTable = NetworkTableInstance.getDefault().getTable(getName());
    m_angleEntry = m_networkTable.getEntry("Angle");
    m_wrappedAngleEntry = m_networkTable.getEntry("Wrapped angle");

    m_turretMaster = new TalonSRX(turretMap.turretID);
    m_turretMaster.configFactoryDefault();

    m_turretMaster.config_IntegralZone(m_positionSlot, 0);
    m_turretMaster.config_kD(m_positionSlot, m_D);
    m_turretMaster.config_kI(m_positionSlot, m_I);
    m_turretMaster.config_kP(m_positionSlot, m_P);
    m_turretMaster.configMotionAcceleration(m_motionAcceleration);
    m_turretMaster.configMotionCruiseVelocity(m_motionCruiseVelocity);

    m_turretMaster.configAllowableClosedloopError(m_positionSlot, toEncoderPulses(m_tolerance));
    m_turretMaster.configClosedLoopPeakOutput(m_positionSlot, 1.0);
    m_turretMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

    m_turretMaster.setNeutralMode(NeutralMode.Brake);
    m_turretMaster.set(ControlMode.PercentOutput, 0);
    m_turretMaster.setSelectedSensorPosition(0);
    m_turretMaster.configForwardSoftLimitThreshold(toEncoderPulses(m_maxTurnDegrees));
    m_turretMaster.configReverseSoftLimitThreshold(toEncoderPulses(-m_maxTurnDegrees));
    // m_turretMaster.configForwardSoftLimitThreshold(toEncoderPulses(m_maxAngle));
    // m_turretMaster.configReverseSoftLimitThreshold(toEncoderPulses(m_minAngle));
    m_turretMaster.configForwardSoftLimitEnable(true);
    m_turretMaster.configReverseSoftLimitEnable(true);
    m_networkTable.getEntry("P gain").setNumber(m_P);
    m_networkTable.getEntry("I gain").setNumber(m_I);
    m_networkTable.getEntry("D gain").setNumber(m_D);
  }

  @Override
  public void periodic() {
    m_networkTable.getEntry("Encoder position").setNumber(getPosition());
    m_P = m_networkTable.getEntry("P gain").getDouble(0.0);
    m_I = m_networkTable.getEntry("I gain").getDouble(0.0);
    m_D = m_networkTable.getEntry("D gain").getDouble(0.0);

    m_angleEntry.setNumber(getAngle());
    m_wrappedAngleEntry.setNumber(DareMathUtil.wrap(getAngle(), m_minAngle, m_maxAngle));
  }

  private int getPosition() {
    return m_turretMaster.getSelectedSensorPosition();
  }

  /**
   * Get the current angle of the turret (CCW positive)
   * @return Angle in degrees
   */
  @Override
  public double getAngle() {
    // Convert from encoder pulses to degrees
    m_logger.log(Level.FINE, "turret position = ", toDegrees(getPosition()));
    return toDegrees(getPosition());
  }

  @Override
  public void resetEncoder() {
    m_turretMaster.setSelectedSensorPosition(0);
  }

  @Override
  public void setSpeed(double speed) {
    m_turretMaster.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void runPosition(double degrees) {
    m_turretMaster.set(ControlMode.MotionMagic, 
      toEncoderPulses(DareMathUtil.wrap(degrees, -180, 180)));
  }

  @Override
  public double wrapDegrees(double degrees) {
    return ((degrees + Math.signum(degrees) * m_maxTurnDegrees) % 360) - Math.signum(degrees) * m_maxTurnDegrees;
  }

  /**
   * Set a target angle for position PID
   * @param angle Angle in degrees
   */
  @Override
  public void setTargetAngle(double angle) {
    m_turretMaster.set(ControlMode.Position, toEncoderPulses(angle));
  }

  private double toDegrees(int encoderPulses) {
    return (double) (encoderPulses / m_encoderResolution) * 360 * m_gearRatio;
  }

  //returns a fused heading problaby
  private int toEncoderPulses(double angle) {
    return (int)((angle / 360) * m_encoderResolution);
  }

  @Override
  public Map<String, Object> getValues() {
    Map<String, Object> values = new HashMap<>();
    values.put("P", m_P);
    values.put("I", m_I);
    values.put("D", m_D);
    return values;
  }
}
