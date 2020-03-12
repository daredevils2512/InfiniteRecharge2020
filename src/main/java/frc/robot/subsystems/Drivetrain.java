/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.logging.*;
import java.util.HashMap;
import java.util.Map;
import java.util.Properties;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
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
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.subsystems.interfaces.IDrivetrain;

/**
 * The drivetrain is a 6 wheel west coast differential drivetrain with two-gear
 * transmission. It consists of four {@link TalonFX} motor controllers for
 * driving (two per side), a {@link DoubleSolenoid} for shifting, two 256PPR
 * optical encoders (one per side) mounted to the output of the gearbox for
 * distance calculation, and a {@link PigeonIMU} for heading calculation.
 */
public class Drivetrain extends PropertySubsystem implements IDrivetrain {
  public static class DrivetrainMap {
    public int driveLeft1ID;
    public int driveLeft2ID;
    public int driveRight1ID;
    public int driveRight2ID;
    public int pigeonID;
    public int driveLeftEncoderChannelA;
    public int driveLeftEncoderChannelB;
    public int driveRightEncoderChannelA;
    public int driveRightEncoderChannelB;
    public int shiftForwardChannel;
    public int shiftReverseChannel;
  }

  /**
   * All network table enties are stored as variables so they can be referenced
   * more reliably (instead of by name via string)
   */
  private final NetworkTable m_networkTable;
  private final NetworkTableEntry m_leftPGainEntry;
  private final NetworkTableEntry m_leftIGainEntry;
  private final NetworkTableEntry m_leftDGainEntry;
  private final NetworkTableEntry m_rightPGainEntry;
  private final NetworkTableEntry m_rightIGainEntry;
  private final NetworkTableEntry m_rightDGainEntry;
  private final NetworkTableEntry m_invertedDrivingEntry;
  private final NetworkTableEntry m_leftDistanceEntry;
  private final NetworkTableEntry m_rightDistanceEntry;
  private final NetworkTableEntry m_leftVelocityEntry;
  private final NetworkTableEntry m_rightVelocityEntry;
  private final NetworkTableEntry m_yawEntry;
  private final NetworkTableEntry m_pitchEntry;
  private final NetworkTableEntry m_rollEntry;
  private final NetworkTableEntry m_fusedHeadingEntry;
  private final NetworkTableEntry m_lowGearEntry;

  private final WPI_TalonFX m_leftDriveMaster;
  private final WPI_TalonFX m_leftDriveFollower;
  private final WPI_TalonFX m_rightDriveMaster;
  private final WPI_TalonFX m_rightDriveFollower;

  private final Encoder m_leftEncoder;
  private final Encoder m_rightEncoder;

  private PigeonIMU m_pigeon;
  private final boolean m_pigeonEnabled;

  private final DoubleSolenoid m_shifter;
  private final Value m_highGearValue = Value.kReverse;
  private final DoubleSolenoid.Value m_lowGearValue = Value.kForward;
  private final boolean m_shiftersEnabled;

  private final int m_encoderResolution;
  private final double m_gearRatio; // Encoder rotations to wheel rotations
  private final double m_wheelDiameter; // Wheel diameter is changed from inches to meters in the getter
  private final double m_wheelCircumference;
  // TODO: Find out track width (can be calculated using the characterization
  // tool)
  private final double m_trackWidth;
  // TODO: Find out max speeds for low and high gear
  private final double m_maxSpeedHighGear; // Max speed in high gear in meters per second
  private final double m_maxSpeedLowGear; // Max speed in low gear in meters per second
  private final double m_maxAngularSpeedHighGear;
  private final double m_maxAngularSpeedLowGear;
  private final double m_maxAcceleration;

  private boolean m_isDrivingInverted = false;

  private final double[] m_gyroData = new double[3]; // Yaw, pitch, and roll in degrees

  private final DifferentialDriveKinematics m_kinematics;
  private final DifferentialDriveOdometry m_odometry;
  private final double m_staticGain;
  private final double m_velocityGain;
  private final double m_accelerationGain;
  // TODO: Tune feedforward values using the characterization tool
  private final SimpleMotorFeedforward m_driveMotorFeedforward;
  private final PIDController m_leftPIDController;
  private final PIDController m_rightPIDController;
  // TODO: Tune velocity PID
  private double m_leftPGain = 0;
  private double m_leftIGain = 0;
  private double m_leftDGain = 0;
  private double m_rightPGain = 0;
  private double m_rightIGain = 0;
  private double m_rightDGain = 0;

  private final Logger m_logger;
  private int logCount = 0;

  /**
   * Creates a new drivetrain
   */
  public Drivetrain(final Properties robotMapProperties) {
    
    m_pigeonEnabled = Boolean.parseBoolean(m_properties.getProperty("pigeonEnabled"));

    m_shiftersEnabled = Boolean.parseBoolean(m_properties.getProperty("shiftersEnabled"));

    m_encoderResolution = Integer.parseInt(m_properties.getProperty("encoderResolution"));
    m_gearRatio = Double.parseDouble(m_properties.getProperty("gearRatio"));
    m_wheelDiameter = Units.inchesToMeters(Double.parseDouble(m_properties.getProperty("wheelDiameter")));
    m_wheelCircumference = Units.inchesToMeters(m_wheelDiameter) * Math.PI;
    m_trackWidth = Units.inchesToMeters(Double.parseDouble(m_properties.getProperty("trackWidth")));

    m_maxSpeedHighGear = Double.parseDouble(m_properties.getProperty("maxSpeedHighGear"));
    m_maxSpeedLowGear = Double.parseDouble(m_properties.getProperty("maxSpeedLowGear"));
    m_maxAngularSpeedHighGear = Double.parseDouble(m_properties.getProperty("maxAngularSpeedHighGear"));
    m_maxAngularSpeedLowGear = Double.parseDouble(m_properties.getProperty("maxAngularSpeedLowGear"));
    m_maxAcceleration = Double.parseDouble(m_properties.getProperty("maxAcceleration"));

    m_staticGain = Double.parseDouble(m_properties.getProperty("staticGain"));
    m_velocityGain = Double.parseDouble(m_properties.getProperty("velocityGain"));
    m_accelerationGain = Double.parseDouble(m_properties.getProperty("accelerationGain"));

    m_leftPGain = Double.parseDouble(m_properties.getProperty("leftPGain"));
    m_leftIGain = Double.parseDouble(m_properties.getProperty("leftIGain"));
    m_leftDGain = Double.parseDouble(m_properties.getProperty("leftDGain"));

    m_rightPGain = Double.parseDouble(m_properties.getProperty("rightPGain"));
    m_rightIGain = Double.parseDouble(m_properties.getProperty("rightIGain"));
    m_rightDGain = Double.parseDouble(m_properties.getProperty("rightDGain"));

    m_networkTable = NetworkTableInstance.getDefault().getTable(getName());
    m_leftPGainEntry = m_networkTable.getEntry("Left P gain");
    m_leftIGainEntry = m_networkTable.getEntry("Left I gain");
    m_leftDGainEntry = m_networkTable.getEntry("Left D gain");
    m_rightPGainEntry = m_networkTable.getEntry("Right P gain");
    m_rightIGainEntry = m_networkTable.getEntry("Right I gain");
    m_rightDGainEntry = m_networkTable.getEntry("Right D gain");
    m_invertedDrivingEntry = m_networkTable.getEntry("Inverted driving");
    m_leftDistanceEntry = m_networkTable.getEntry("Left encoder distance");
    m_rightDistanceEntry = m_networkTable.getEntry("Right encoder distance");
    m_leftVelocityEntry = m_networkTable.getEntry("Left encoder velocity entry");
    m_rightVelocityEntry = m_networkTable.getEntry("Right encoder velocity entry");
    m_yawEntry = m_networkTable.getEntry("Yaw");
    m_pitchEntry = m_networkTable.getEntry("Pitch");
    m_rollEntry = m_networkTable.getEntry("Roll");
    m_fusedHeadingEntry = m_networkTable.getEntry("Gyro fused heading");
    m_lowGearEntry = m_networkTable.getEntry("Low gear");

    m_logger = Logger.getLogger(Drivetrain.class.getName());

    m_leftDriveMaster = new WPI_TalonFX(getInteger(robotMapProperties.getProperty("driveLeft1ID")));
    m_leftDriveFollower = new WPI_TalonFX(getInteger(robotMapProperties.getProperty("driveLeft2ID")));
    m_rightDriveMaster = new WPI_TalonFX(getInteger(robotMapProperties.getProperty("driveRight1ID")));
    m_rightDriveFollower = new WPI_TalonFX(getInteger(robotMapProperties.getProperty("driveRight2ID")));

    // Config to factory defaults to prevent unexpected behavior
    m_leftDriveMaster.configFactoryDefault();
    m_leftDriveFollower.configFactoryDefault();
    m_rightDriveMaster.configFactoryDefault();
    m_rightDriveFollower.configFactoryDefault();

    final SupplyCurrentLimitConfiguration supplyCurrentLimitConfig = new SupplyCurrentLimitConfiguration();
    supplyCurrentLimitConfig.enable = true;
    supplyCurrentLimitConfig.currentLimit = 35;
    supplyCurrentLimitConfig.triggerThresholdCurrent = 60;
    supplyCurrentLimitConfig.triggerThresholdTime = 0.3;

    final StatorCurrentLimitConfiguration statorCurrentLimitConfiguration = new StatorCurrentLimitConfiguration();
    statorCurrentLimitConfiguration.enable = true;
    statorCurrentLimitConfiguration.currentLimit = 65;
    statorCurrentLimitConfiguration.triggerThresholdCurrent = 100;
    statorCurrentLimitConfiguration.triggerThresholdTime = 1;

    m_leftDriveMaster.configSupplyCurrentLimit(supplyCurrentLimitConfig);
    m_leftDriveFollower.configSupplyCurrentLimit(supplyCurrentLimitConfig);
    m_rightDriveMaster.configSupplyCurrentLimit(supplyCurrentLimitConfig);
    m_rightDriveFollower.configSupplyCurrentLimit(supplyCurrentLimitConfig);

    m_leftDriveMaster.configStatorCurrentLimit(statorCurrentLimitConfiguration);
    m_leftDriveFollower.configStatorCurrentLimit(statorCurrentLimitConfiguration);
    m_rightDriveMaster.configStatorCurrentLimit(statorCurrentLimitConfiguration);
    m_rightDriveFollower.configStatorCurrentLimit(statorCurrentLimitConfiguration);

    // Designate drive masters
    m_leftDriveFollower.follow(m_leftDriveMaster);
    m_rightDriveFollower.follow(m_rightDriveMaster);

    m_leftDriveMaster.setInverted(InvertType.InvertMotorOutput);
    m_leftDriveFollower.setInverted(InvertType.FollowMaster);
    m_rightDriveMaster.setInverted(InvertType.None);
    m_rightDriveFollower.setInverted(InvertType.FollowMaster);

    m_leftDriveMaster.setNeutralMode(NeutralMode.Coast);
    m_leftDriveFollower.setNeutralMode(NeutralMode.Coast);
    m_rightDriveMaster.setNeutralMode(NeutralMode.Coast);
    m_rightDriveFollower.setNeutralMode(NeutralMode.Coast);

    m_leftEncoder = new Encoder(getInteger(robotMapProperties.getProperty("driveLeftEncoderChannelA")), getInteger(robotMapProperties.getProperty("driveLeftEncoderChannelB")));
    m_rightEncoder = new Encoder(getInteger(robotMapProperties.getProperty("driveRightEncoderChannelA")), getInteger(robotMapProperties.getProperty("driveRightEncoderChannelB")));
    double distancePerPules = m_wheelCircumference * m_gearRatio / m_encoderResolution;
    m_leftEncoder.setDistancePerPulse(distancePerPules);
    m_leftEncoder.setReverseDirection(true);
    m_rightEncoder.setDistancePerPulse(distancePerPules);

    if (m_pigeonEnabled) {
      m_pigeon = new PigeonIMU(getInteger(robotMapProperties.getProperty("pigeonID")));
      m_pigeon.configFactoryDefault();
      m_pigeon.setYaw(0.0);
      m_pigeon.setFusedHeading(0.0);
    }

    m_shifter = m_shiftersEnabled
        ? new DoubleSolenoid(getInteger(robotMapProperties.getProperty("drivetrainShiftForwardChannel")), getInteger(robotMapProperties.getProperty("drivetrainShiftReverseChannel")))
        : null;
    if (m_shiftersEnabled) {this.setLowGear(false);}

    m_leftDriveMaster.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 10, 0.5));
    m_leftDriveFollower.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 10, 0.5));
    m_rightDriveMaster.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 10, 0.5));
    m_rightDriveFollower.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 10, 0.5));

    m_kinematics = new DifferentialDriveKinematics(m_trackWidth);
    m_driveMotorFeedforward = new SimpleMotorFeedforward(m_staticGain, m_velocityGain, m_accelerationGain);
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
    logCount++;
    if (logCount%20 == 0) {
      m_logger.log(Level.INFO, m_rightDriveMaster.getFaults(new Faults()).toString());
      m_logger.log(Level.INFO, m_leftDriveMaster.getFaults(new Faults()).toString());
    }
    m_leftPGainEntry.setNumber(m_leftPGain);
    m_leftIGainEntry.setNumber(m_leftIGain);
    m_leftDGainEntry.setNumber(m_leftDGain);
    m_rightPGainEntry.setNumber(m_leftPGain);
    m_rightIGainEntry.setNumber(m_leftIGain);
    m_rightDGainEntry.setNumber(m_leftDGain);

    m_networkTable.getEntry("heading").setDouble(getHeading());
    m_invertedDrivingEntry.setBoolean(m_isDrivingInverted);
    m_leftDistanceEntry.setNumber(getLeftDistance());
    m_rightDistanceEntry.setNumber(getRightDistance());
    m_leftVelocityEntry.setNumber(getLeftVelocity());
    m_rightVelocityEntry.setNumber(getRightVelocity());
    m_fusedHeadingEntry.setNumber(getFusedHeading());
    m_yawEntry.setNumber(getYaw());
    m_pitchEntry.setNumber(getPitch());
    m_rollEntry.setNumber(getRoll());
    m_lowGearEntry.setBoolean(getLowGear());
  }

  /**
   * Get the current max speed depending on the current gear
   * 
   * @return Speed in meters per second
   */
  @Override
  public double getMaxSpeed() {
    return getLowGear() ? m_maxSpeedLowGear : m_maxSpeedHighGear;
  }

  @Override
  public double getMaxAngularSpeed() {
    return getLowGear() ? m_maxAngularSpeedLowGear : m_maxAngularSpeedHighGear;
  }

  /**
   * in meters per second per second
   * 
   * @return m/s^2
   */
  @Override
  public double getMaxAcceleration() {
    return m_maxAcceleration;
  }

  private void resetEncoders() {
    m_logger.fine("encoders reset");
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  @Override
  public boolean getDrivingInverted() {
    return m_isDrivingInverted;
  }

  @Override
  public void setDrivingInverted(final boolean wantsInverted) {
    m_logger.fine("inverted to" + wantsInverted);
    m_isDrivingInverted = wantsInverted;
  }

  @Override
  public double getLeftDistance() {
    return m_leftEncoder.getDistance();
  }

  @Override
  public double getRightDistance() {
    return m_rightEncoder.getDistance();
  }

  @Override
  public double getAverageDistance() {
    return (getLeftDistance() + getRightDistance()) / 2;
   }

  @Override
  public double getLeftVelocity() {
    return m_leftEncoder.getRate();
  }

  @Override
  public double getRightVelocity() {
    return m_rightEncoder.getRate();
  }

  @Override
  public DifferentialDriveKinematics getKinematics() {
    return m_kinematics;
  }

  private double getYaw() {
    m_logger.log(Level.FINE, "yaw = ", m_gyroData[0]);
    return m_gyroData[0];
  }

  private double getPitch() {
    m_logger.log(Level.FINE, "pitch = ", m_gyroData[1]);
    return m_gyroData[1];
  }

  private double getRoll() {
    m_logger.log(Level.FINE, "roll = ", m_gyroData[2]);
    return m_gyroData[2];
  }

  private double getFusedHeading() {
    if (m_pigeonEnabled)
      m_logger.fine("fused heading = " + m_pigeon.getFusedHeading());
    return m_pigeonEnabled ? m_pigeon.getFusedHeading() : 0.0;
  }

  @Override
  public double getHeading() {
    return m_pigeonEnabled ? -getFusedHeading() % 360 : 0.0;
  }

  /**
   * Must be called periodically to retrieve gyro data from the {@link PigeonIMU}
   */
  private void updateGyroData() {
    if (m_pigeonEnabled) {
      m_pigeon.getYawPitchRoll(m_gyroData);
    }
  }

  @Override
  public void setLowGear(final boolean wantsLowGear) {
    if (m_shiftersEnabled) {
      m_shifter.set(wantsLowGear ? m_lowGearValue : m_highGearValue);
    } else {
      m_logger.info("shifters disabled");
    }
  }

  @Override
  public boolean getLowGear() {
    return m_shiftersEnabled ? m_shifter.get() == m_lowGearValue : false;
  }

  /**
   * Get the current pose (rotation and translation) of the robot
   * 
   * @return Pose with translation in meters
   */
  @Override
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  @Override
  public void resetPose() {
    resetEncoders();
    final Pose2d newPose = new Pose2d();
    m_odometry.resetPosition(newPose, Rotation2d.fromDegrees(getFusedHeading()));
    m_logger.fine("reset pose");
  }

  @Override
  public void resetPose(final Translation2d translation) {
    resetEncoders();
    final Pose2d newPose = new Pose2d(translation, getPose().getRotation());
    m_odometry.resetPosition(newPose, Rotation2d.fromDegrees(getFusedHeading()));
    m_logger.fine("reset pose");
  }

  @Override
  public void resetPose(final Rotation2d rotation) {
    resetEncoders();
    final Pose2d newPose = new Pose2d(getPose().getTranslation(), rotation);
    m_odometry.resetPosition(newPose, Rotation2d.fromDegrees(getFusedHeading()));
    m_logger.fine("reset pose");
  }

  @Override
  public void resetPose(final Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getFusedHeading()));
    m_logger.fine("reset pose");
  }

  /**
   * Must be called periodically to maintain an accurate position and heading
   */
  private void updateOdometry() {
    m_odometry.update(Rotation2d.fromDegrees(getFusedHeading()), m_leftEncoder.getDistance(),
        m_rightEncoder.getDistance());
  }

  @Override
  public void simpleArcadeDrive(double move, double turn) {
    move = 0.75 * move;
    turn = 0.75 * turn;
    m_leftDriveMaster.set(ControlMode.PercentOutput, move - turn);
    m_rightDriveMaster.set(ControlMode.PercentOutput, move + turn);
  }

  @Override
  public void voltageTank(final double left, final double right) {
    m_leftDriveMaster.setVoltage(left);
    m_rightDriveMaster.setVoltage(right);
  }

  /**
   * Set the drivetrain's linear and angular target velocities
   * 
   * @param velocity        Velocity in meters per second
   * @param angularVelocity Angular velocity in radians per second
   */
  @Override
  public void velocityArcadeDrive(double velocity, final double angularVelocity) {
    velocity = m_isDrivingInverted ? -velocity : velocity;
    setSpeeds(m_kinematics.toWheelSpeeds(new ChassisSpeeds(velocity, 0, angularVelocity)));
  }

  @Override
  public void setWheelSpeeds(final double left, final double right) {
    setSpeeds(new DifferentialDriveWheelSpeeds(left, right));
  }

  private void setSpeeds(final DifferentialDriveWheelSpeeds wheelSpeeds) {
    final double leftFeedforward = m_driveMotorFeedforward.calculate(wheelSpeeds.leftMetersPerSecond);
    final double rightFeedforward = m_driveMotorFeedforward.calculate(wheelSpeeds.rightMetersPerSecond);
    final double leftPIDOutput = m_leftPIDController.calculate(m_leftEncoder.getRate(), wheelSpeeds.leftMetersPerSecond);
    final double rightPIDOutput = m_rightPIDController.calculate(m_rightEncoder.getRate(), wheelSpeeds.rightMetersPerSecond);

    m_leftDriveMaster.set(leftFeedforward + leftPIDOutput);
    m_rightDriveMaster.set(rightFeedforward + rightPIDOutput);
  }

  @Override
  public Map<String, Object> getValues() {
    final Map<String, Object> values = new HashMap<>();
    values.put("leftPGain", m_leftPGain);
    values.put("leftIGain", m_leftIGain);
    values.put("leftDGain", m_leftDGain);
    values.put("rightPGain", m_rightPGain);
    values.put("rightIGain", m_rightIGain);
    values.put("rightDGain", m_rightDGain);
    m_logger.fine("put values");
    return values;
  }

  @Override
  public SimpleMotorFeedforward getFeedForward() {
    return m_driveMotorFeedforward;
  }

  @Override
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
  }

  @Override
  public PIDController getLeftController() {
    return m_leftPIDController;
  }

  @Override
  public PIDController getRightController() {
    return m_rightPIDController;
  }

  
}