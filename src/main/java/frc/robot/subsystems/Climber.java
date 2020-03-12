package frc.robot.subsystems;

import java.util.Map;
import java.util.Properties;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.subsystems.interfaces.IClimber;

public class Climber extends PropertySubsystem implements IClimber {  

  //TODO: max left climber encoder value = -2578 min = 235 min while retracted = 311


  private final Boolean m_shiftersEnabled;
  private final Boolean m_encodersEnabled;

  private final WPI_TalonSRX m_leftClimbMotor;
  private final WPI_TalonSRX m_rightClimbMotor;

  private final Encoder m_leftEncoder;
  private final Encoder m_rightEncoder;

  private final double m_gearRatio = 1.0 / 30.0;

  private final DoubleSolenoid m_climberExtender;
  private final Value m_retracted = Value.kReverse;
  private final Value m_extended = Value.kForward;

  private final DoubleSolenoid m_climbShifter;
  private final Value m_climbing = Value.kForward;
  private final Value m_driving = Value.kReverse;

  private final NetworkTable m_networktable;
  private final NetworkTableEntry m_leftClimberEncoderEntry;
  private final NetworkTableEntry m_rightClimberEncoderEntry;
  private final NetworkTableEntry m_resetEncoderEntry;


  private final int m_maxClimberPos = -2578;
  private final int m_minClimberPos = 0;

  public Climber(Properties robotMapProperties) {
    m_shiftersEnabled = getBoolean(m_properties.getProperty("shiftersEnabled"));
    m_encodersEnabled = getBoolean(m_properties.getProperty("encodersEnabled"));

    m_networktable = NetworkTableInstance.getDefault().getTable(getName());
    m_leftClimberEncoderEntry = m_networktable.getEntry("left climber encoder");
    m_rightClimberEncoderEntry = m_networktable.getEntry("right climber encoder");

    m_leftClimbMotor = new WPI_TalonSRX(getInteger(robotMapProperties.getProperty("climberLeftID")));
    m_rightClimbMotor = new WPI_TalonSRX(getInteger(robotMapProperties.getProperty("climberRightID")));
    
    m_leftEncoder = m_encodersEnabled ? new Encoder(getInteger(robotMapProperties.getProperty("climberLeftEncoderChannelA")),
      getInteger(robotMapProperties.getProperty("climberLeftEncoderChannelB"))) : null;
    m_rightEncoder = m_encodersEnabled ? new Encoder(getInteger(robotMapProperties.getProperty("climberRightEncoderChannelA")),
      getInteger(robotMapProperties.getProperty("climberRightEncoderChannelB"))) : null;

    m_climberExtender = m_shiftersEnabled ? new DoubleSolenoid(getInteger(robotMapProperties.getProperty("climberExtenderForwardID")),
        getInteger(robotMapProperties.getProperty("climberExtenderReverseID"))) : null;
    m_climbShifter = m_shiftersEnabled ? new DoubleSolenoid(getInteger(robotMapProperties.getProperty("shifterPortForwardID")),
        getInteger(robotMapProperties.getProperty("shifterPortReverseID"))) : null;
    
    m_resetEncoderEntry = m_networktable.getEntry("Reset encoder");
  }

  @Override
  public void periodic() {
    m_leftClimberEncoderEntry.setDouble(m_leftEncoder.get());
    m_rightClimberEncoderEntry.setDouble(m_rightEncoder.get());
    m_networktable.getEntry("climbers raised").setBoolean(getExtended());

    if (m_resetEncoderEntry.getBoolean(false)) {
      resetEncoders();
      m_resetEncoderEntry.setBoolean(false);
    }
  }

  @Override
  public void extendClimbers(double leftSpeed, double rightSpeed) {
    extendLeftClimber(leftSpeed);
    extendRightClimber(rightSpeed);
    m_logger.fine("left speed = " + leftSpeed + "right speed = " + rightSpeed);
  }



  public void extendLeftClimber(double speed) {
    if (speed < 0 && m_leftEncoder.get() > m_minClimberPos) {
      speed = 0;
    } else if (speed > 0 && m_leftEncoder.get() < m_maxClimberPos) {
      speed = 0;
    }
    m_logger.info("climber speed = " + speed);
    m_leftClimbMotor.set(speed);
  }

  public void extendRightClimber(double speed) {
    if (speed < 0 && m_rightEncoder.get() > m_minClimberPos) {
      speed = 0;
    } else if (speed > 0 && m_rightEncoder.get() < m_maxClimberPos) {
      speed = 0;
    }
    m_rightClimbMotor.set(speed);
  }

  @Override
  public void toggleClimberExtended() {
    raiseClimbers(!getExtended());
  }

  @Override
  public void raiseClimbers(boolean wantsExtended) {
    m_logger.info("extended to " + wantsExtended);
    if (m_shiftersEnabled) {
      m_climberExtender.set(wantsExtended ? m_extended : m_retracted);
    }
  }
  
  @Override
  public void shiftToClimbing(boolean wantsClimbing) {
    m_logger.info("shifted to climbing " + wantsClimbing);
    if (m_shiftersEnabled) {
      m_climbShifter.set(wantsClimbing ? m_climbing : m_driving);
    }
  }

  public boolean getExtended() {
    return m_shiftersEnabled ? m_climberExtender.get() == m_climbing : false;
  }

  public boolean getShifted() {
    return m_shiftersEnabled ? m_climbShifter.get() == m_climbing : false;
  }

  @Override
  public void resetEncoders() {
    if (m_encodersEnabled) {
      m_leftEncoder.reset();
      m_rightEncoder.reset();
    }
  }

  @Override
  public int getLeftEncoder() {
    return m_encodersEnabled ? m_leftEncoder.get() : 0;
  }

  @Override
  public int getRightEncoder() {
    return m_encodersEnabled ? m_rightEncoder.get() : 0;
  }

  @Override
  public Map<String, Object> getValues() {
    return null;
  }

}