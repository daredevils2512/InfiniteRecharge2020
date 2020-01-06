/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// TODO: Determine whether we will be using built in or external encoders and adjust accordingly
public class Drivetrain extends SubsystemBase {
  private final int m_encoderPulsesPerRevolution = 4096; // TODO: Check drivetrain PPR
  private final double m_wheelDiameterInches = 8; // TODO: Check wheel diameter
  private final double m_wheelCircumferenceInches = m_wheelDiameterInches * Math.PI;

  // TODO: Configure drivetrain CAN
  private final int m_leftDrive1ID = 1;
  private final int m_leftDrive2ID = 2;
  private final int m_rightDrive1ID = 3;
  private final int m_rightDrive2ID = 4;
  private final int m_pigeonID = 5;
  
  private double m_leftEncoderLastTotalInches = 0;
  private double m_rightEncoderLastTotalInches = 0;

  private final WPI_TalonFX m_leftDrive1;
  private final WPI_TalonFX m_leftDrive2;
  private final WPI_TalonFX m_rightDrive1;
  private final WPI_TalonFX m_rightDrive2;
  private final PigeonIMU m_pigeon; 
  private final DifferentialDrive m_differentialDrive;

  private final int m_shifterForwardChannel = 0;
  private final int m_shifterReverseChannel = 1;
  private final DoubleSolenoid m_shifter;
  private final DoubleSolenoid.Value m_highGearValue = Value.kForward;
  private final DoubleSolenoid.Value m_lowGearValue = Value.kReverse;
  private final double m_highGearRatio = 1;
  private final double m_lowGearRatio = 1;

  /**
   * Creates a new Drivetrain.
   */
  public Drivetrain() {
    m_leftDrive1 = new WPI_TalonFX(m_leftDrive1ID);
    m_leftDrive2 = new WPI_TalonFX(m_leftDrive2ID);
    m_rightDrive1 = new WPI_TalonFX(m_rightDrive1ID);
    m_rightDrive2 = new WPI_TalonFX(m_rightDrive2ID);

    // Config to factory defaults to prevent unexpected behavior
    m_leftDrive1.configFactoryDefault();
    m_leftDrive2.configFactoryDefault();
    m_rightDrive1.configFactoryDefault();
    m_rightDrive2.configFactoryDefault();

    // Designate drive masters
    m_leftDrive2.follow(m_leftDrive1);
    m_rightDrive2.follow(m_rightDrive1);

    m_leftDrive1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    m_rightDrive1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    m_differentialDrive = new DifferentialDrive(m_leftDrive1, m_rightDrive1);

    m_pigeon = new PigeonIMU(m_pigeonID);
    m_pigeon.configFactoryDefault();

    m_shifter = new DoubleSolenoid(m_shifterForwardChannel, m_shifterReverseChannel);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left drive distance", getLeftDistance());
    SmartDashboard.putNumber("Right drive distance", getRightDistance());
    SmartDashboard.putNumber("Left drive velocity", getLeftVelocity());
    SmartDashboard.putNumber("Right drive velocity", getRightVelocity());
    SmartDashboard.putBoolean("Low gear", getLowGear());
  }

  public void arcadeDrive(double move, double turn) {
    m_differentialDrive.arcadeDrive(move, turn);
  }

  public void setLowGear(boolean wantsLowGear) {
    // If the gear is changing, store current drive distances
    // because the conversion cannot be done after shifting
    if(wantsLowGear != getLowGear()) {
      m_leftEncoderLastTotalInches += getLeftDistanceSinceShift();
      m_rightEncoderLastTotalInches += getRightDistanceSinceShift();
      m_leftDrive1.setSelectedSensorPosition(0);
      m_rightDrive1.setSelectedSensorPosition(0);
    }
    m_shifter.set(wantsLowGear ? m_lowGearValue : m_highGearValue);
  }

  public boolean getLowGear() {
    return m_shifter.get() == m_lowGearValue;
  }

  public void resetDriveEncoders() {
    m_leftEncoderLastTotalInches = 0;
    m_rightEncoderLastTotalInches = 0;
    m_leftDrive1.setSelectedSensorPosition(0);
    m_rightDrive1.setSelectedSensorPosition(0);
  }

  /**
   * Returns the encoder pulses measured by the left drive master since last shift or reset
   */
  private int getLeftPosition() {
    return m_leftDrive1.getSelectedSensorPosition();
  }

  /**
   * Returns the encoder pulses measured by the right drive master since last shift or reset
   */
  private int getRightPosition() {
    return m_rightDrive1.getSelectedSensorPosition();
  }

  /**
   * Returns the distance traveled by the left drive master since last shift or reset
   * @return Distance in inches
   */
  private double getLeftDistanceSinceShift() {
    return toInches(getLeftPosition(), getLowGear());
  }

  /**
   * Returns the distance traveled by the right drive master since last shift or reset
   * @return Distance in inches
   */
  private double getRightDistanceSinceShift() {
    return toInches(getRightPosition(), getLowGear());
  }

  /**
   * Returns the distance in inches traveled by the left drive master since last shift or reset
   */
  public double getLeftDistance() {
    return m_leftEncoderLastTotalInches + getLeftDistanceSinceShift();
  }

  public double getRightDistance() {
    return m_rightEncoderLastTotalInches + getRightDistanceSinceShift();
  }

  /**
   * Get left drive velocity (measured by encoder)
   * @return Velocity in inches per second
   */
  public double getLeftVelocity() {
    // Convert to seconds (from 100ms) and then to inches (from sensor units)
    return toInches(m_leftDrive1.getSelectedSensorVelocity() * 10, getLowGear());
  }

  /**
   * Get right drive velocity (measured by encoder)
   * @return Velocity in inches per second
   */
  public double getRightVelocity() {
    // Convert to seconds (from 100ms) and then to inches (from sensor units)
    return toInches(m_rightDrive1.getSelectedSensorVelocity() * 10, getLowGear());
  }

  /**
   * Convert from drive encoder pulses to inches
   * <p> Because shifting changes the gearing of the drivetrain
   * (assuming the encoder is installed on gearing input rather than output),
   * this conversion depends on the current gear and does not work for
   * encoder measurments accumulated over one or more gear changes
   */
  private double toInches(int encoderPulses, boolean inLowGear) {
    return encoderPulses / m_encoderPulsesPerRevolution * (inLowGear ? m_lowGearRatio : m_highGearRatio) * m_wheelCircumferenceInches;
  }
}
