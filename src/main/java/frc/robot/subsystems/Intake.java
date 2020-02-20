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
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.sensors.LimitSwitch;

public class Intake extends PropertySubsystem {
  private final NetworkTable m_networkTable;
  private final NetworkTableEntry m_extendedEntry;
  private final NetworkTableEntry m_motionMagicEnbledEntry;
  private final NetworkTableEntry m_encoderTicksEntry;
  private final NetworkTableEntry m_angleEntry;
  private final NetworkTableEntry m_pGainEntry;
  private final NetworkTableEntry m_iGainEntry;
  private final NetworkTableEntry m_dGainEntry;
  private final NetworkTableEntry m_arbitraryFeedforwardEntry;
  private final NetworkTableEntry m_cruiseVelocityEntry;
  private final NetworkTableEntry m_accelerationEntry;
  private final NetworkTableEntry m_runMotorOutputEntry;
  private final NetworkTableEntry m_extendMotorOutputEntry;
  private final NetworkTableEntry m_resetEncoderEntry;

  private final int m_runMotorID;
  private final WPI_TalonSRX m_runMotor;

  private final int m_extendMotorID;
  private final WPI_TalonSRX m_extendMotor;

  private boolean m_retractedLimitSwitchEnabled;
  private boolean m_extendedLimitSwitchEnabled;

  private final int m_retractedLimitSwitchPort;
  private final int m_extendedLimitSwitchPort;
  private final LimitSwitch m_retractedLimitSwitch;
  private final LimitSwitch m_extendedLimitSwitch;

  private final int m_extenderEncoderResolution;
  private final double m_extenderGearRatio; // TODO: Find intake extender gear ratio
  // TODO: Find the intake range of motion
  private final double m_extendedAngle; // Angle in degrees
  private final double m_retractedAngle;

  // TODO: Configure PID for intake extender
  private final int m_motionMagicSlot;
  private double m_pGain;
  private double m_iGain;
  private double m_dGain;
  private double m_arbitraryFeedforward;
  private double m_cruiseVelocity;
  private double m_acceleration;

  private boolean m_extended = false;

  private boolean m_motionMagicEnabled = false;

  /**
   * Creates a new power cell intake
   */
  public Intake() {
    super(Intake.class.getName());
    
    m_networkTable = NetworkTableInstance.getDefault().getTable(getName());
    m_extendedEntry = m_networkTable.getEntry("Extended");
    m_motionMagicEnbledEntry = m_networkTable.getEntry("Motion magic enabled");
    m_encoderTicksEntry = m_networkTable.getEntry("Encoder ticks");
    m_angleEntry = m_networkTable.getEntry("Angle");
    m_pGainEntry = m_networkTable.getEntry("P gain");
    m_iGainEntry = m_networkTable.getEntry("I gain");
    m_dGainEntry = m_networkTable.getEntry("D gain");
    m_arbitraryFeedforwardEntry = m_networkTable.getEntry("Arbitrary feedforward");
    m_cruiseVelocityEntry = m_networkTable.getEntry("Cruise velocity");
    m_accelerationEntry = m_networkTable.getEntry("Acceleration");
    m_runMotorOutputEntry = m_networkTable.getEntry("Run motor output");
    m_extendMotorOutputEntry = m_networkTable.getEntry("Extend motor output");
    m_resetEncoderEntry = m_networkTable.getEntry("Reset encoder");

    m_runMotorID = Integer.parseInt(properties.getProperty("runMotorID"));
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
    m_arbitraryFeedforward = Double.parseDouble(properties.getProperty("arbitraryFeedforward"));
    m_cruiseVelocity = Double.parseDouble(properties.getProperty("cruiseVelocity"));
    m_acceleration = Double.parseDouble(properties.getProperty("acceleration"));

    m_runMotor = new WPI_TalonSRX(m_runMotorID);
    m_runMotor.configFactoryDefault();
    
    m_runMotor.setInverted(InvertType.InvertMotorOutput);
    m_runMotor.setNeutralMode(NeutralMode.Brake);

    m_extendMotor = new WPI_TalonSRX(m_extendMotorID);
    m_extendMotor.configFactoryDefault();

    // Config PID for extender
    m_extendMotor.config_kP(m_motionMagicSlot, m_pGain);
    m_extendMotor.config_kI(m_motionMagicSlot, m_iGain);
    m_extendMotor.config_kD(m_motionMagicSlot, m_dGain);
    m_extendMotor.configMotionCruiseVelocity(toEncoderTicksPer100Milliseconds(m_cruiseVelocity));
    m_extendMotor.configMotionAcceleration(toEncoderTicksPer100MillisecondsPerSecond(m_acceleration));

    m_extendMotor.setInverted(InvertType.InvertMotorOutput);
    m_extendMotor.setNeutralMode(NeutralMode.Coast);
    m_extendMotor.setSensorPhase(false);
    m_extendMotor.setSelectedSensorPosition(toEncoderTicks(m_retractedAngle));

    if (m_retractedLimitSwitchEnabled) {
      m_retractedLimitSwitch = new LimitSwitch(m_retractedLimitSwitchPort);
    } else {
      m_retractedLimitSwitch = null;
    }
    if (m_extendedLimitSwitchEnabled) {
      m_extendedLimitSwitch = new LimitSwitch(m_extendedLimitSwitchPort);
    } else {
      m_extendedLimitSwitch = null;
    }
  }

  @Override
  public void periodic() {
    m_pGain = m_pGainEntry.getNumber(m_pGain).doubleValue();
    m_iGain = m_iGainEntry.getNumber(m_iGain).doubleValue();
    m_dGain = m_dGainEntry.getNumber(m_dGain).doubleValue();
    m_arbitraryFeedforward = m_arbitraryFeedforwardEntry.getNumber(m_arbitraryFeedforward).doubleValue();
    m_cruiseVelocity = m_cruiseVelocityEntry.getNumber(m_cruiseVelocity).doubleValue();
    m_acceleration = m_accelerationEntry.getNumber(m_acceleration).doubleValue();

    m_extendMotor.config_kP(m_motionMagicSlot, m_pGain);
    m_extendMotor.config_kI(m_motionMagicSlot, m_iGain);
    m_extendMotor.config_kD(m_motionMagicSlot, m_dGain);
    m_extendMotor.configMotionCruiseVelocity(toEncoderTicksPer100Milliseconds(m_cruiseVelocity));
    m_extendMotor.configMotionAcceleration(toEncoderTicksPer100MillisecondsPerSecond(m_acceleration));

    if (m_resetEncoderEntry.getBoolean(false)) {
      resetIntakeExtenderAngle();
    }

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
      // Up is 0 degrees (gravity scalar is 0) and down is ~90 degrees (gravity scalar
      // is 1)
      double gravityScalar = Math.sin(Math.toRadians(targetAngle));
      m_extendMotor.set(ControlMode.MotionMagic, targetPosition,
        DemandType.ArbitraryFeedForward, m_arbitraryFeedforward * gravityScalar);
    }

    m_extendedEntry.setBoolean(m_extended);
    m_motionMagicEnbledEntry.setBoolean(m_motionMagicEnabled);
    m_encoderTicksEntry.setNumber(m_extendMotor.getSelectedSensorPosition());
    m_angleEntry.setNumber(toDegrees(m_extendMotor.getSelectedSensorPosition()));
    m_pGainEntry.setNumber(m_pGain);
    m_iGainEntry.setNumber(m_iGain);
    m_dGainEntry.setNumber(m_dGain);
    m_arbitraryFeedforwardEntry.setNumber(m_arbitraryFeedforward);
    m_cruiseVelocityEntry.setNumber(m_cruiseVelocity);
    m_accelerationEntry.setNumber(m_acceleration);
    m_runMotorOutputEntry.setNumber(m_runMotor.getMotorOutputPercent());
    m_extendMotorOutputEntry.setNumber(m_extendMotor.getMotorOutputPercent());
    m_resetEncoderEntry.setBoolean(false);
  }

  public void runIntake(double speed) {
    m_runMotor.set(ControlMode.PercentOutput, speed);
  }

  public boolean getMotionMagicEnabled() {
    return m_motionMagicEnabled;
  }

  public void setMotionMagicEnabled(boolean wantsEnabled) {
    if (!wantsEnabled) {
      m_extendMotor.set(ControlMode.PercentOutput, 0);
    }
    m_motionMagicEnabled = wantsEnabled;
  }

  public void toggleMotionMagicEnabled() {
    setMotionMagicEnabled(!getMotionMagicEnabled());
  }

  public void resetIntakeExtenderAngle() {
    m_extendMotor.setSelectedSensorPosition(0);
  }

  public boolean getExtended() {
    if (m_extended)
      logger.fine("intake extended");
    return m_extended;
  }

  public void setExtended(boolean wantsExtended) {
    m_extended = wantsExtended;
  }

  public void toggleExtended() {
    setExtended(!getExtended());
  }

  /**
   * Temporary function for testing/tuning the extender
   */
  public void runExtender(double output) {
    if (m_motionMagicEnabled) {
      return;
    }
    
    if (m_retractedLimitSwitchEnabled) {
      if (m_retractedLimitSwitch.get()) {
        output = Math.max(output, 0);
      }
    }
    if (m_extendedLimitSwitchEnabled) {
      if (m_extendedLimitSwitch.get()) {
        output = Math.min(output, 0);
      }
    }

    m_extendMotor.set(ControlMode.PercentOutput, output);
  }

  /**
   * Convert from raw sensor units to an angle in degrees
   * <p>
   * Applies only the the extender
   * 
   * @param sensorUnits
   * @return Angle in degrees
   */
  private double toDegrees(int sensorUnits) {
    return (double)sensorUnits / m_extenderEncoderResolution / m_extenderGearRatio * 360;
  }

  /**
   * Convert from an angle in degrees to raw sensor units
   * <p>
   * Applies only to the extender
   * 
   * @param degrees
   * @return
   */
  private int toEncoderTicks(double degrees) {
    return (int)(degrees / 360 * m_extenderGearRatio * m_extenderEncoderResolution);
  }

  private int toEncoderTicksPer100Milliseconds(double degreesPerSecond) {
    double encoderTicksPerSecond = degreesPerSecond / 360 * m_extenderGearRatio * m_extenderEncoderResolution;
    return (int)(encoderTicksPerSecond / 10);
  }

  private int toEncoderTicksPer100MillisecondsPerSecond(double degreesPerSecondPerSecond) {
    double encoderTicksPerSecondPerSecond = degreesPerSecondPerSecond / 360 * m_extenderGearRatio * m_extenderEncoderResolution;
    return (int)(encoderTicksPerSecondPerSecond / 10);
  }

  @Override
  protected Map<String, Object> getValues() {
    Map<String, Object> values = new HashMap<>();
    values.put("pGain", m_pGain);
    values.put("iGain", m_iGain);
    values.put("dGain", m_dGain);
    values.put("arbitraryFeedforward", m_arbitraryFeedforward);
    values.put("cruiseVelocity", m_cruiseVelocity);
    values.put("acceleration", m_acceleration);
    return values;
  }
}
