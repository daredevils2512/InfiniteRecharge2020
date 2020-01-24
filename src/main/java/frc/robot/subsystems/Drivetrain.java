/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  private final NetworkTable m_networkTable;

  // TODO: Configure drivetrain CAN
  private final int m_leftDriveMasterID = -1;
  private final int m_leftDrive1ID = -1;
  private final int m_rightDriveMasterID = -1;
  private final int m_rightDrive1ID = -1;
  private final int m_pigeonID = -1;

  private final WPI_TalonFX m_leftDriveMaster;
  private final WPI_TalonFX m_leftDrive1;
  private final WPI_TalonFX m_rightDriveMaster;
  private final WPI_TalonFX m_rightDrive1;
  private final DifferentialDrive m_differentialDrive;

  private final int m_shifterForwardChannel = 0;
  private final int m_shifterReverseChannel = 1;
  private final DoubleSolenoid m_shifter;
  private final DoubleSolenoid.Value m_highGearValue = Value.kForward;
  private final DoubleSolenoid.Value m_lowGearValue = Value.kReverse;

  private final PigeonIMU m_pigeon;

  private final int m_encoderPulsesPerRevolution = 2048; // TODO: Check drivetrain encoder PPR
  private final double m_gearRatio = 3;
  private final double m_wheelDiameterInches = 8; // TODO: Check wheel diameter
  private final double m_wheelCircumferenceInches = m_wheelDiameterInches * Math.PI;

  private double[] m_yprZero;

  private boolean m_isDrivingInverted = false;

  /**
   * Creates a new Drivetrain.
   */
  public Drivetrain() {
    m_networkTable = NetworkTableInstance.getDefault().getTable(getName());

    m_leftDriveMaster = new WPI_TalonFX(m_leftDriveMasterID);
    m_leftDrive1 = new WPI_TalonFX(m_leftDrive1ID);
    m_rightDriveMaster = new WPI_TalonFX(m_rightDriveMasterID);
    m_rightDrive1 = new WPI_TalonFX(m_rightDrive1ID);

    // Config to factory defaults to prevent unexpected behavior
    m_leftDriveMaster.configFactoryDefault();
    m_leftDrive1.configFactoryDefault();
    m_rightDriveMaster.configFactoryDefault();
    m_rightDrive1.configFactoryDefault();

    // Designate drive masters
    m_leftDrive1.follow(m_leftDriveMaster);
    m_rightDrive1.follow(m_rightDriveMaster);

    m_leftDriveMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    m_rightDriveMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    m_differentialDrive = new DifferentialDrive(m_leftDriveMaster, m_rightDriveMaster);
    
    m_pigeon = new PigeonIMU(m_pigeonID);
    m_pigeon.configFactoryDefault();

    m_shifter = new DoubleSolenoid(m_shifterForwardChannel, m_shifterReverseChannel);
  }

  @Override
  public void periodic() {
    m_networkTable.getEntry("Left distance").setNumber(getLeftDistance());
    m_networkTable.getEntry("Right distance").setNumber(getLeftDistance());
    m_networkTable.getEntry("Left velocity").setNumber(getLeftVelocity());
    m_networkTable.getEntry("Right velocity").setNumber(getLeftVelocity());
    m_networkTable.getEntry("Inverted driving").setBoolean(m_isDrivingInverted);
    m_networkTable.getEntry("Low gear").setBoolean(getLowGear());
    m_networkTable.getEntry("Yaw").setNumber(getYaw());
    m_networkTable.getEntry("Pitch").setNumber(getPitch());
    m_networkTable.getEntry("Roll").setNumber(getRoll());
  }

  public void arcadeDrive(final double move, final double turn) {
    m_differentialDrive.arcadeDrive(move, turn);
  }

  public void driveLeft(final double speed) {
    m_differentialDrive.tankDrive(speed, 0);
  }

  public void driveRight(final double speed) {
    m_differentialDrive.tankDrive(speed, 0);
  }

  /**
   * Drive arcade style using percent output
   * 
   * <p>Alternative to using the {@link DifferentialDrive} provided by WPI.
   * This gives the subsystem more control over the implimentation and configuration
   * of arcade drive. For example, {@link TalonSRX} allows the use of velocity
   * PID in place of standard percent output.
   * @param move Forwards/backwards move speed (-1.0 to 1.0)
   * @param turn Speed bias for left/right side (-1.0 to 1.0)
   */
  public void arcadeDriveAlt(double move, double turn) {
    // Reverse direction if inverted driving is enabled
    move = m_isDrivingInverted ? -move : move;
    m_leftDriveMaster.set(ControlMode.PercentOutput, move, DemandType.ArbitraryFeedForward, turn);
    m_rightDriveMaster.set(ControlMode.PercentOutput, move, DemandType.ArbitraryFeedForward, -turn);
  }

  public void driveLeftAlt(final double speed) {
    m_leftDriveMaster.set(ControlMode.PercentOutput, speed);
  }

  public void driveRightAlt(final double speed) {
    m_rightDriveMaster.set(ControlMode.PercentOutput, speed);
  }

  public void setDrivingInverted(boolean wantsInverted) {
    m_isDrivingInverted = wantsInverted;
  }

  public void setLowGear(final boolean wantsLowGear) {
    m_shifter.set(wantsLowGear ? m_lowGearValue : m_highGearValue);
  }

  public boolean getLowGear() {
    return m_shifter.get() == m_lowGearValue;
  }

  public void resetDriveEncoders() {
    m_leftDriveMaster.setSelectedSensorPosition(0);
    m_rightDriveMaster.setSelectedSensorPosition(0);
  }

  private double[] getYawPitchRoll() {
    double[] ypr = null;
    m_pigeon.getYawPitchRoll(ypr);
    return ypr;
  }

  public double getYaw() {
    return getYawPitchRoll()[0] - m_yprZero[0];
  }

  public double getPitch() {
    return getYawPitchRoll()[1] - m_yprZero[1];
  }

  public double getRoll() {
    return getYawPitchRoll()[2] - m_yprZero[2];
  }

  // There might be a better way to do this
  public void resetGyro() {
    m_yprZero = getYawPitchRoll();
  }

  /**
   * Get the distance traveled by the left side of the drivetrain
   * @return Distance in inches
   */
  public double getLeftDistance() {
    return toInches(m_leftDriveMaster.getSelectedSensorPosition());
  }

  /**
   * Get the distance traveled by the right side of the drivetrain
   * @return Distance in inches
   */
  public double getRightDistance() {
    return toInches(m_rightDriveMaster.getSelectedSensorPosition());
  }

  /**
   * Get left drive velocity (measured by encoder)
   * 
   * @return Velocity in inches per second
   */
  public double getLeftVelocity() {
    // Convert to seconds (from 100ms) and then to inches (from sensor units)
    return toInches(m_leftDrive1.getSelectedSensorVelocity() * 10);
  }

  /**
   * Get right drive velocity (measured by encoder)
   * 
   * @return Velocity in inches per second
   */
  public double getRightVelocity() {
    // Convert to seconds (from 100ms) and then to inches (from sensor units)
    return toInches(m_rightDrive1.getSelectedSensorVelocity() * 10);
  }

  /**
   * Convert from drive encoder pulses to inches
   */
  private double toInches(final int encoderPulses) {
    return encoderPulses / m_encoderPulsesPerRevolution * m_gearRatio * m_wheelCircumferenceInches;
  }

  /**
   * Convert from inches to encoder pulses
   */
  private int toEncoderPulses(final int inches) {
    return (int)(inches / m_wheelCircumferenceInches / m_gearRatio * m_encoderPulsesPerRevolution);
  }
}
