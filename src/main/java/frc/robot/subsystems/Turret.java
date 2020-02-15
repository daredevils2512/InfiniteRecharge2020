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
import java.util.logging.Level;
import java.util.logging.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
  private static Logger logger = Logger.getLogger(Turret.class.getName());
  private final NetworkTable m_networkTable;
  private final Properties properties;
  private static final String PROPERTIES_NAME = "/turret.properties";

  private final int m_turretMasterID; // TODO: Configure CAN on turret
  private final TalonSRX m_turretMaster;

  // TODO: Find encoder and gearing details for turret
  private final double m_encoderResolution;
  private final double m_gearRatio;

  // TODO: Tune position PID
  private final int m_positionSlot;
  private double m_P = 0;
  private double m_I = 0;
  private double m_D = 0;

  /**
   * Creates a new turret
   */
  public Turret() {
    Properties defaultProperties = new Properties();
    properties = new Properties(defaultProperties);
    try {
      InputStream deployStream = new FileInputStream(Filesystem.getDeployDirectory() + PROPERTIES_NAME);
      InputStream robotStream = new FileInputStream(Filesystem.getOperatingDirectory() + PROPERTIES_NAME);
      defaultProperties.load(deployStream);
      properties.load(robotStream);
      logger.info("succesfuly loaded");
    } catch(IOException e) {
      logger.log(Level.WARNING, "failed to load", e);
    }

    m_turretMasterID = Integer.parseInt(properties.getProperty("turretMasterID"));

    m_encoderResolution = Integer.parseInt(properties.getProperty("encoderResolution"));
    m_gearRatio = Double.parseDouble(properties.getProperty("gearRatio"));

    m_positionSlot = Integer.parseInt(properties.getProperty("positionSlot"));
    m_P = Double.parseDouble(properties.getProperty("P"));
    m_I = Double.parseDouble(properties.getProperty("I"));
    m_D = Double.parseDouble(properties.getProperty("D"));

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
  }

  private int getPosition() {
    return m_turretMaster.getSelectedSensorPosition();
  }

  /**
   * Get the current angle of the turret
   * @return Angle in degrees
   */
  public double getAngle() {
    // Convert from encoder pulses to degrees
    logger.log(Level.FINE, "turret position = ", toDegrees(getPosition()));
    return toDegrees(getPosition());
  }

  public void resetEncoder() {
    m_turretMaster.setSelectedSensorPosition(0);
  }

  public void setSpeed(double speed) {
    m_turretMaster.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Set a target angle for position PID
   * @param angle Angle in degrees
   */
  public void setTargetAngle(double angle) {
    m_turretMaster.set(ControlMode.Position, toEncoderPulses(angle));
  }

  private double toDegrees(int encoderPulses) {
    return (double) (encoderPulses / m_encoderResolution) * 360 * m_gearRatio;
  }

  //returns a fused heading problaby
  private double toEncoderPulses(double angle) {
    return (int) (angle / 360) * m_encoderResolution;
  }

  private void savePID() {
    try {
      OutputStream outputStream = new FileOutputStream(Filesystem.getOperatingDirectory() + PROPERTIES_NAME);
      properties.setProperty("P", "" + m_P);
      properties.setProperty("I", "" + m_I);
      properties.setProperty("D", "" + m_D);
      properties.store(outputStream, "saved pId or somethinges");
    } catch(IOException e) {
      e.printStackTrace();
    }
  }
}
