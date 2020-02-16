/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.Properties;
import java.util.logging.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.PhotoEye;

public class Queue extends SubsystemBase {
  private static Logger logger = Logger.getLogger(Queue.class.getName());
  
  private final int m_photoEyeChannel;
  private final PhotoEye m_photoEye;

  public final NetworkTable m_networkTable;
  public final NetworkTableEntry m_isClosedEntry;
  private final NetworkTableEntry m_runSpeedEntry;
  private final Properties properties;
  private static final String PROPERTIES_NAME = "/queue.properties";

  private final int m_runMotorID;
  private final TalonSRX m_runMotor;

  private final boolean m_gateEnabled;
  // TODO: Check all the gate wiring and stuff
  private final int m_gateForwardChannel;
  private final int m_gateReverseChannel;
  private final DoubleSolenoid.Value m_openValue = Value.kForward;
  private final DoubleSolenoid.Value m_closedValue = Value.kReverse;
  private final DoubleSolenoid m_gate;

  /**
   * Creates a new Queue.
   */
  public Queue() {
    m_networkTable = NetworkTableInstance.getDefault().getTable(getName());
    m_runSpeedEntry = m_networkTable.getEntry("Run speed");
    m_isClosedEntry = m_networkTable.getEntry("Is closed");

    // Properties defaultProperties = new Properties();
    properties = new Properties();
    try {
      InputStream deployStream = new FileInputStream(Filesystem.getDeployDirectory() + PROPERTIES_NAME);
      // InputStream robotStream = new FileInputStream(Filesystem.getOperatingDirectory() + PROPERTIES_NAME);
      // defaultProperties.load(deployStream);
      properties.load(deployStream);
      logger.info("succesfuly loaded");
    } catch(IOException e) {
      logger.log(Level.SEVERE, "failed to load", e);
    }

    m_photoEyeChannel = Integer.parseInt(properties.getProperty("photoEyeChannel"));

    m_runMotorID = Integer.parseInt(properties.getProperty("runMotorID"));

    m_runMotor = new TalonSRX(m_runMotorID);
    m_runMotor.configFactoryDefault();

    m_photoEye = new PhotoEye(m_photoEyeChannel);

    m_gateEnabled = Boolean.parseBoolean(properties.getProperty("gateEnabled"));
    m_gateForwardChannel = Integer.parseInt(properties.getProperty("gateForwardChannel"));
    m_gateReverseChannel = Integer.parseInt(properties.getProperty("gateReverseChannel"));
    m_gate = m_gateEnabled ? new DoubleSolenoid(m_gateForwardChannel, m_gateReverseChannel) : null;
  }

  @Override
  public void periodic() {
    m_runSpeedEntry.setNumber(m_runMotor.getMotorOutputPercent());
    m_isClosedEntry.setBoolean(getClosed());
  }

  public void run(double speed) {
    m_runMotor.set(ControlMode.PercentOutput, speed);
  }

  public void run(double speed, boolean wantsClosed) {
    setClosed(wantsClosed);
    run(speed);
  }

  public boolean getClosed() {
    if (m_gateEnabled) {
      return m_gate.get() == m_closedValue;
    } else {
      return false;
    }
  }

  public void setClosed(boolean wantsClosed) {
    if (m_gateEnabled) {
      m_gate.set(wantsClosed ? m_closedValue : m_openValue);
    }
  }

  public boolean hasPowerCell() {
    return m_photoEye.get();
  }
}
