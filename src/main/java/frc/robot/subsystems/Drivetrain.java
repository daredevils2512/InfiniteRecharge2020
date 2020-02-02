/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  /**
   * All network table enties are stored as variables so they
   * can be referenced more reliably (instead of by name via string)
   */
  private final NetworkTable m_networkTable;
  private final NetworkTableEntry m_leftPGainEntry;
  private final NetworkTableEntry m_leftIGainEntry;
  private final NetworkTableEntry m_leftDGainEntry;
  private final NetworkTableEntry m_rightPGainEntry;
  private final NetworkTableEntry m_rightIGainEntry;
  private final NetworkTableEntry m_rightDGainEntry;
  public final NetworkTableEntry m_invertedDrivingEntry;
  public final NetworkTableEntry m_lowGearEntry;
  public final NetworkTableEntry m_yawEntry;
  public final NetworkTableEntry m_pitchEntry;
  public final NetworkTableEntry m_rollEntry;

  private final int m_leftDriveMasterID = 10;
  private final int m_leftDriveFollowerID = 11;
  private final int m_rightDriveMasterID = 12;
  private final int m_rightDriveFollowerID = 13;

  private final WPI_TalonFX m_leftDriveMaster;
  private final WPI_TalonFX m_leftDriveFollower;
  private final WPI_TalonFX m_rightDriveMaster;
  private final WPI_TalonFX m_rightDriveFollower;

  private final int m_leftEncoderChannelA = -1;
  private final int m_leftEncoderChannelB = -1;
  private final int m_rightEncoderChannelA = -1;
  private final int m_rightEncoderChannelB = -1;

  private final Encoder m_leftEncoder;
  private final Encoder m_rightEncoder;

  private final int m_pigeonID = 14;
  private final PigeonIMU m_pigeon;

  private final int m_shifterForwardChannel = -1;
  private final int m_shifterReverseChannel = -1;
  private final DoubleSolenoid m_shifter;
  private final DoubleSolenoid.Value m_highGearValue = Value.kForward;
  private final DoubleSolenoid.Value m_lowGearValue = Value.kReverse;

  private final int m_encoderResolution = 256;
  private final double m_gearRatio = 3 / 1;
  private final double m_wheelDiameter = Units.inchesToMeters(6); // Wheel diameter in meters
  // TODO: Find out track width (can be calculated using the characterization tool)
  private final double m_trackWidth = Units.inchesToMeters(28);
  // TODO: Find out max speeds for low and high gear
  private final double m_maxSpeedHighGear = 3; // Max speed in high gear in meters per second
  private final double m_maxSpeedLowGear = 1; // Max speed in low gear in meters per second

  private boolean m_isDrivingInverted = false;

  private double[] m_gyroData = new double[3]; // Yaw, pitch, and roll in degrees

  private final DifferentialDriveKinematics m_kinematics;
  private final DifferentialDriveOdometry m_odometry;
  // TODO: Tune feedforward values using the characterization tool
  private final SimpleMotorFeedforward m_driveMotorFeedforward = new SimpleMotorFeedforward(1, 3);
  private final PIDController m_leftPIDController;
  private final PIDController m_rightPIDController;
  // TODO: Tune velocity PID
  private double m_leftPGain = 0;
  private double m_leftIGain = 0;
  private double m_leftDGain = 0;
  private double m_rightPGain = 0;
  private double m_rightIGain = 0;
  private double m_rightDGain = 0;

  /**
   * Creates a new Drivetrain.
   */
  public Drivetrain() {
    m_networkTable = NetworkTableInstance.getDefault().getTable(getName());
    m_leftPGainEntry = m_networkTable.getEntry("Left P gain");
    m_leftIGainEntry = m_networkTable.getEntry("Left I gain");
    m_leftDGainEntry = m_networkTable.getEntry("Left D gain");
    m_rightPGainEntry = m_networkTable.getEntry("Right P gain");
    m_rightIGainEntry = m_networkTable.getEntry("Right I gain");
    m_rightDGainEntry = m_networkTable.getEntry("Right D gain");
    m_invertedDrivingEntry = m_networkTable.getEntry("Inverted driving");
    m_lowGearEntry = m_networkTable.getEntry("Low gear");
    m_yawEntry = m_networkTable.getEntry("Yaw");
    m_pitchEntry = m_networkTable.getEntry("Pitch");
    m_rollEntry = m_networkTable.getEntry("Roll");

    m_leftDriveMaster = new WPI_TalonFX(m_leftDriveMasterID);
    m_leftDriveFollower = new WPI_TalonFX(m_leftDriveFollowerID);
    m_rightDriveMaster = new WPI_TalonFX(m_rightDriveMasterID);
    m_rightDriveFollower = new WPI_TalonFX(m_rightDriveFollowerID);

    // Config to factory defaults to prevent unexpected behavior
    m_leftDriveMaster.configFactoryDefault();
    m_leftDriveFollower.configFactoryDefault();
    m_rightDriveMaster.configFactoryDefault();
    m_rightDriveFollower.configFactoryDefault();

    // Designate drive masters
    m_leftDriveFollower.follow(m_leftDriveMaster);
    m_rightDriveFollower.follow(m_rightDriveMaster);

    m_leftDriveMaster.setInverted(InvertType.InvertMotorOutput);
    m_leftDriveFollower.setInverted(InvertType.FollowMaster);
    m_rightDriveMaster.setInverted(InvertType.None);
    m_rightDriveFollower.setInverted(InvertType.FollowMaster);

    m_leftEncoder = new Encoder(m_leftEncoderChannelA, m_leftEncoderChannelB);
    m_rightEncoder = new Encoder(m_rightEncoderChannelA, m_rightEncoderChannelB);
    m_leftEncoder.setDistancePerPulse(m_wheelDiameter * Math.PI * m_gearRatio / m_encoderResolution);
    m_rightEncoder.setDistancePerPulse(m_wheelDiameter * Math.PI * m_gearRatio / m_encoderResolution);

    m_pigeon = new PigeonIMU(m_pigeonID);
    m_pigeon.configFactoryDefault();
    m_pigeon.setFusedHeading(0);

    m_shifter = new DoubleSolenoid(m_shifterForwardChannel, m_shifterReverseChannel);

    m_kinematics = new DifferentialDriveKinematics(m_trackWidth);
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getYaw()));
    m_leftPIDController = new PIDController(m_leftPGain, m_leftIGain, m_leftDGain);
    m_rightPIDController = new PIDController(m_rightPGain, m_rightIGain, m_rightDGain);
  }

  @Override
  public void periodic() {
    m_leftPGain = m_leftPGainEntry.getNumber(m_leftPGain).doubleValue();
    m_leftIGain = m_leftIGainEntry.getNumber(m_leftIGain).doubleValue();
    m_leftDGain = m_leftDGainEntry.getNumber(m_leftDGain).doubleValue();
    m_rightPGain = m_rightPGainEntry.getNumber(m_rightPGain).doubleValue();
    m_rightIGain = m_rightIGainEntry.getNumber(m_rightIGain).doubleValue();
    m_rightDGain = m_rightDGainEntry.getNumber(m_rightDGain).doubleValue();

    m_leftPIDController.setPID(m_leftPGain, m_leftIGain, m_leftDGain);
    m_rightPIDController.setPID(m_rightPGain, m_rightIGain, m_rightDGain);

    updateGyroData();
    updateOdometry();

    m_leftPGainEntry.setNumber(m_leftPGain);
    m_leftIGainEntry.setNumber(m_leftIGain);
    m_leftDGainEntry.setNumber(m_leftDGain);
    m_rightPGainEntry.setNumber(m_leftPGain);
    m_rightIGainEntry.setNumber(m_leftIGain);
    m_rightDGainEntry.setNumber(m_leftDGain);

    m_invertedDrivingEntry.setBoolean(m_isDrivingInverted);
    m_lowGearEntry.setBoolean(getLowGear());
    m_yawEntry.setNumber(getYaw());
    m_pitchEntry.setNumber(getPitch());
    m_rollEntry.setNumber(getRoll());
  }

  /**
   * Set the drivetrain's linear and angular target velocities
   * @param velocity Velocity in meters per second
   * @param angularVelocity Angular velocity in radians per second
   */
  public void arcadeDrive(double velocity, double angularVelocity) {
    velocity = m_isDrivingInverted ? -velocity : velocity;
    setSpeeds(m_kinematics.toWheelSpeeds(new ChassisSpeeds(velocity, 0, angularVelocity)));
  }

  private void setSpeeds(DifferentialDriveWheelSpeeds wheelSpeeds) {
    double leftFeedforward = m_driveMotorFeedforward.calculate(wheelSpeeds.leftMetersPerSecond);
    double rightFeedforward = m_driveMotorFeedforward.calculate(wheelSpeeds.rightMetersPerSecond);
    double leftPIDOutput = m_leftPIDController.calculate(m_leftEncoder.getRate(), wheelSpeeds.leftMetersPerSecond);
    double rightPIDOutput = m_rightPIDController.calculate(m_rightEncoder.getRate(), wheelSpeeds.rightMetersPerSecond);
    
    m_leftDriveMaster.set(leftFeedforward + leftPIDOutput);
    m_rightDriveMaster.set(rightFeedforward + rightPIDOutput);
  }

  public void setDrivingInverted(boolean wantsInverted) {
    m_isDrivingInverted = wantsInverted;
  }

  /**
   * Get the current max speed depending on the current gear
   * @return Speed in meters per second
   */
  public double getMaxSpeed() {
    return getLowGear() ? m_maxSpeedLowGear : m_maxSpeedHighGear;
  }

  public void setLowGear(final boolean wantsLowGear) {
    m_shifter.set(wantsLowGear ? m_lowGearValue : m_highGearValue);
  }

  public boolean getLowGear() {
    return m_shifter.get() == m_lowGearValue;
  }

  public void resetDriveEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  /**
   * Must be called periodically to retrieve gyro data from the {@link PigeonIMU}
   */
  private void updateGyroData() {
    m_pigeon.getYawPitchRoll(m_gyroData);
  }

  public double getYaw() {
    return m_gyroData[0];
  }

  public double getPitch() {
    return m_gyroData[1];
  }

  public double getRoll() {
    return m_gyroData[2];
  }

  /**
   * Set a new heading for the drivetrain
   * @param angle
   */
  public void setHeading(double angle) {
    m_pigeon.setFusedHeading(angle);
  }

  /**
   * Must be called periodically to maintain an accurate position and heading
   */
  private void updateOdometry() {
    m_odometry.update(Rotation2d.fromDegrees(getYaw()), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  }

  /**
   * Get the current pose (rotation and translation) of the robot
   * @return Pose with x and y in meters
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
}
