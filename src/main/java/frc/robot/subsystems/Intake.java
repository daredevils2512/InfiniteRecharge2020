/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.Properties;
import java.util.logging.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.LimitSwitch;
import frc.robot.utils.PropertyFiles;

public class Intake extends SubsystemBase {
  private static final Logger logger = Logger.getLogger(Intake.class.getName());

  private final NetworkTable m_networkTable;
  private final Properties properties;
  private static final String NAME = "intake";

  private final NetworkTableEntry m_extendedEntry;
  private final NetworkTableEntry m_motionMagicEnbledEntry;
  private final NetworkTableEntry m_angleEntry;
  private final NetworkTableEntry m_pGainEntry;
  private final NetworkTableEntry m_iGainEntry;
  private final NetworkTableEntry m_dGainEntry;
  private final NetworkTableEntry m_arbitraryFeedforwardEntry;

  private final int m_extendMotorID;
  private final TalonSRX m_extendMotor;

  private boolean m_retractedLimitSwitchEnabled;
  private boolean m_extendedLimitSwitchEnabled;

  private final int m_retractedLimitSwitchPort;
  private final int m_extendedLimitSwitchPort;
  private final LimitSwitch m_retractedLimitSwitch;
  private final LimitSwitch m_extendedLimitSwitch;

  private final int m_extenderEncoderResolution;
  private final double m_extenderGearRatio; // TODO: Find intake extender gear ratio
  // TODO: Find the intake range of motion
  private final double m_extendedAngle; // Angle in degrees, assuming retracted is zero degrees
  private final double m_retractedAngle;

  // TODO: Configure PID for intake extender
  private final int m_motionMagicSlot;
  private double m_pGain;
  private double m_iGain;
  private double m_dGain;
  private double m_arbitraryFeedForward;

  private boolean m_extended = false;

  private boolean m_motionMagicEnabled = false;
  
  /**
   * Creates a new power cell intake
   */
  public Intake() {
    properties = PropertyFiles.loadProperties(NAME);
    
    m_networkTable = NetworkTableInstance.getDefault().getTable(getName());
    m_extendedEntry = m_networkTable.getEntry("Extended");
    m_motionMagicEnbledEntry = m_networkTable.getEntry("Motion magic enabled");
    m_angleEntry = m_networkTable.getEntry("Angle");
    m_pGainEntry = m_networkTable.getEntry("P gain");
    m_iGainEntry = m_networkTable.getEntry("I gain");
    m_dGainEntry = m_networkTable.getEntry("D gain");
    m_arbitraryFeedforwardEntry = m_networkTable.getEntry("Arbitrary feedforward");

    m_extendMotorID = Integer.parseInt(properties.getProperty("extendMotorID"));

    m_retractedLimitSwitchEnabled = Boolean.parseBoolean(properties.getProperty("retractedLimitSwitchEnabled"));
    m_extendedLimitSwitchEnabled = Boolean.parseBoolean(properties.getProperty("extendedLimitSwitchEnabled"));

    m_retractedLimitSwitchPort = Integer.parseInt(properties.getProperty("retractedLimitSwitchPort"));
    m_extendedLimitSwitchPort = Integer.parseInt(properties.getProperty("extendedLimitSwitchPort"));

    m_extenderEncoderResolution = Integer.parseInt(properties.getProperty("extenderEncoderResolution"));
    m_extenderGearRatio = Double.parseDouble(properties.getProperty("extenderGearRatio"));
    m_extendedAngle = Double.parseDouble(properties.getProperty("extendedAngle"));
    m_retractedAngle = Double.parseDouble(properties.getProperty("retractedAngle"));

    m_motionMagicSlot = Integer.parseInt(properties.getProperty("motionMagicSlot"));
    m_pGain = Double.parseDouble(properties.getProperty("pGain"));
    m_iGain = Double.parseDouble(properties.getProperty("iGain"));
    m_dGain = Double.parseDouble(properties.getProperty("dGain"));
    m_arbitraryFeedForward = Double.parseDouble(properties.getProperty("arbitraryFeedForward"));

    m_extendMotor = new TalonSRX(m_extendMotorID);
    m_extendMotor.configFactoryDefault();

    // Config PID for extender
    m_extendMotor.config_kP(m_motionMagicSlot, m_pGain);
    m_extendMotor.config_kI(m_motionMagicSlot, m_iGain);
    m_extendMotor.config_kD(m_motionMagicSlot, m_dGain);

    if (m_retractedLimitSwitchEnabled) {m_retractedLimitSwitch = new LimitSwitch(m_retractedLimitSwitchPort);
    } else {m_retractedLimitSwitch = null;}
    if (m_extendedLimitSwitchEnabled) {m_extendedLimitSwitch = new LimitSwitch(m_extendedLimitSwitchPort);
    } else {m_extendedLimitSwitch = null;}
  }

  @Override
  public void periodic() {
    m_pGain = m_pGainEntry.getNumber(m_pGain).doubleValue();
    m_iGain = m_pGainEntry.getNumber(m_iGain).doubleValue();
    m_dGain = m_pGainEntry.getNumber(m_dGain).doubleValue();
    m_arbitraryFeedForward = m_pGainEntry.getNumber(m_arbitraryFeedForward).doubleValue();
    
    m_extendMotor.config_kP(m_motionMagicSlot, m_pGain);
    m_extendMotor.config_kI(m_motionMagicSlot, m_iGain);
    m_extendMotor.config_kD(m_motionMagicSlot, m_dGain);

    if (m_extendedLimitSwitchEnabled) {  
      if (m_extendedLimitSwitch.get()) {
        m_extendMotor.setSelectedSensorPosition(toEncoderTicks(m_extendedAngle));
      }
    }
    if (m_retractedLimitSwitchEnabled) {
      if (m_retractedLimitSwitch.get()) {
        m_extendMotor.setSelectedSensorPosition(toEncoderTicks(m_retractedAngle));
      }
    }
    if (m_motionMagicEnabled) {
      double targetAngle = m_extended ? m_extendedAngle : 0;
      double targetPosition = toEncoderTicks(targetAngle);
      // Up is 0 degrees (gravity scalar is 0) and down is ~90 degrees (gravity scalar is 1)
      double gravityScalar = Math.sin(Math.toRadians(targetAngle));
      m_extendMotor.set(ControlMode.MotionMagic, targetPosition, DemandType.ArbitraryFeedForward, m_arbitraryFeedForward * gravityScalar);
    }

    m_extendedEntry.setBoolean(m_extended);
    m_motionMagicEnbledEntry.setBoolean(m_motionMagicEnabled);
    m_angleEntry.setNumber(toDegrees(m_extendMotor.getSelectedSensorPosition()));
    m_pGainEntry.setNumber(m_pGain);
    m_iGainEntry.setNumber(m_iGain);
    m_dGainEntry.setNumber(m_dGain);
    m_arbitraryFeedforwardEntry.setNumber(m_arbitraryFeedForward);
  }

  public void setMotionMagicEnabled(boolean wantsEnabled) {
    if (!wantsEnabled) {
      m_extendMotor.set(ControlMode.PercentOutput, 0);
    }
    m_motionMagicEnabled = wantsEnabled;
  }

  public void resetIntakeExtenderAngle() {
    m_extendMotor.setSelectedSensorPosition(0);
  }

  public boolean getExtended() {
    if (m_extended) logger.fine("intake extended");
    return m_extended;
  }

  public void setExtended(boolean wantsExtended) {
    m_extended = wantsExtended;
  }

  /**
   * Temporary function for testing/tuning the extender
   */
  public void runExtender(double output) {
    // Stop running motion magic so it doesn't interfere
    m_motionMagicEnabled = false;

    if (m_extendedLimitSwitch.get()) {
      output = Math.min(output, 0);
      m_extendMotor.setSelectedSensorPosition(toEncoderTicks(m_extendedAngle));
    }

    m_extendMotor.set(ControlMode.PercentOutput, output);
  }

  /**
   * Convert from raw sensor units to an angle in degrees
   * <p>Applies only the the extender
   * @param sensorUnits
   * @return Angle in degrees
   */
  private double toDegrees(int sensorUnits) {
    return (double)sensorUnits / m_extenderEncoderResolution * m_extenderGearRatio * 360;
  }

  /**
   * Convert from an angle in degrees to raw sensor units
   * <p>Applies only to the extender
   * @param degrees
   * @return
   */
  private int toEncoderTicks(double degrees) {
    return (int)(degrees / 360 / m_extenderGearRatio * m_extenderEncoderResolution);
  }

  public void savePID() {
    PropertyFiles.saveProperties(properties, new Double[]{m_pGain, m_iGain, m_dGain}, new String[]{"pGain", "iGain", "dGain"}, NAME);
  }
}
