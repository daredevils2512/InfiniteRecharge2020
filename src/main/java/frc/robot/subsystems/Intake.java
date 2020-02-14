/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.logging.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.LimitSwitch;

public class Intake extends SubsystemBase {
  private static final Logger logger = Logger.getLogger(Intake.class.getName());

  private final NetworkTable m_networkTable;
  private final NetworkTableEntry m_extendedEntry;
  private final NetworkTableEntry m_motionMagicEnbledEntry;
  private final NetworkTableEntry m_angleEntry;

  private final int m_extendMotorID = 20;
  private final int m_runMotorID = 21;
  private final TalonSRX m_extendMotor;
  private final TalonSRX m_runMotor;

  private final int m_retractedLimitSwitchPort = -1;
  private final int m_extendedLimitSwitchPort = -1;
  private final LimitSwitch m_retractedLimitSwitch;
  private final LimitSwitch m_extendedLimitSwitch;

  private final int m_extenderEncoderResolution = 4096;
  private final double m_extenderGearRatio = 1; // TODO: Find intake extender gear ratio
  // TODO: Find the intake range of motion
  private final double m_extendedAngle = 0; // Angle in degrees, assuming retracted is zero degrees

  // TODO: Configure PID for intake extender
  private final int m_motionMagicSlot = 0;
  private final double m_pGain = 0;
  private final double m_iGain = 0;
  private final double m_dGain = 0;
  private final double m_arbitraryFeedForward = 0;

  private boolean m_extended = false;

  private boolean m_motionMagicEnabled = false;
  
  /**
   * Creates a new power cell intake
   */
  public Intake() {
    m_networkTable = NetworkTableInstance.getDefault().getTable(getName());
    m_extendedEntry = m_networkTable.getEntry("Extended");
    m_motionMagicEnbledEntry = m_networkTable.getEntry("Motion magic enabled");
    m_angleEntry = m_networkTable.getEntry("Angle");

    m_extendMotor = new TalonSRX(m_extendMotorID);
    m_runMotor = new TalonSRX(m_runMotorID);
    m_extendMotor.configFactoryDefault();
    m_runMotor.configFactoryDefault();

    // Config PID for extender
    m_extendMotor.config_kP(m_motionMagicSlot, m_pGain);
    m_extendMotor.config_kI(m_motionMagicSlot, m_iGain);
    m_extendMotor.config_kD(m_motionMagicSlot, m_dGain);

    m_retractedLimitSwitch = new LimitSwitch(m_retractedLimitSwitchPort);
    m_extendedLimitSwitch = new LimitSwitch(m_extendedLimitSwitchPort);
  }

  @Override
  public void periodic() {
    if (m_retractedLimitSwitch.get()) {
      m_extendMotor.setSelectedSensorPosition(0);
    } else if (m_extendedLimitSwitch.get()) {
      m_extendMotor.setSelectedSensorPosition(toEncoderTicks(m_extendedAngle));
    } else if (m_motionMagicEnabled) {
      double targetAngle = m_extended ? m_extendedAngle : 0;
      double targetPosition = toEncoderTicks(targetAngle);
      double gravityScalar = Math.cos(Math.toRadians(targetAngle));
      m_extendMotor.set(ControlMode.MotionMagic, targetPosition, DemandType.ArbitraryFeedForward, m_arbitraryFeedForward * gravityScalar);
    }

    m_extendedEntry.setBoolean(m_extended);
    m_motionMagicEnbledEntry.setBoolean(m_motionMagicEnabled);
    m_angleEntry.setNumber(toDegrees(m_extendMotor.getSelectedSensorPosition()));
  }

  public void setMotionMagicEnabled(boolean wantsEnabled) {
    if (!wantsEnabled) {
      m_extendMotor.set(ControlMode.PercentOutput, 0);
    }

    m_motionMagicEnabled = wantsEnabled;
  }

  public boolean getExtended() {
    if (m_extended) logger.fine("intake extended");
    return m_extended;
  }

  public void setExtended(boolean wantsExtended) {
    m_extended = wantsExtended;
  }

  public void run(double speed) {
    m_runMotor.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Temporary function for testing/tuning the extender
   */
  public void runExtender(double output) {
    // Stop running motion magic so it doesn't interfere
    m_motionMagicEnabled = false;

    if (m_retractedLimitSwitch.get()) {
      output = Math.max(0, output);
    } else if (m_extendedLimitSwitch.get()) {
      output = Math.min(output, 0);
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
}
