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
  private final Boolean m_shiftersEnabled;
  private final Boolean m_encodersEnabled;

  private final WPI_TalonSRX m_leftClimbMotor;
  private final WPI_TalonSRX m_rightClimbMotor;

  private final Encoder m_leftEncoder;
  private final Encoder m_rightEncoder;

  private final DoubleSolenoid m_climberExtender;
  private final Value m_retracted = Value.kReverse;
  private final Value m_extended = Value.kForward;

  private final NetworkTable m_networktable;
  private final NetworkTableEntry m_leftClimberEncoderEntry;
  private final NetworkTableEntry m_rightClimberEncoderEntry;

  public Climber(Properties robotMapProperties) {
    m_shiftersEnabled = Boolean.parseBoolean(m_properties.getProperty("shiftersEnabled"));
    m_encodersEnabled = Boolean.parseBoolean(m_properties.getProperty("encodersEnabled"));

    m_networktable = NetworkTableInstance.getDefault().getTable(getName());
    m_leftClimberEncoderEntry = m_networktable.getEntry("left climber encoder");
    m_rightClimberEncoderEntry = m_networktable.getEntry("right climber encoder");

    m_leftClimbMotor = new WPI_TalonSRX(getInteger(robotMapProperties.getProperty("climberLeftID")));
    m_rightClimbMotor = new WPI_TalonSRX(getInteger(robotMapProperties.getProperty("climberRightID")));
     
    m_leftEncoder = new Encoder(getInteger(robotMapProperties.getProperty("climberLeftEncoderChannelA")),
      getInteger(robotMapProperties.getProperty("climberLeftEncoderChannelB")));
    m_rightEncoder = new Encoder(getInteger(robotMapProperties.getProperty("climberRightEncoderChannelA")),
      getInteger(robotMapProperties.getProperty("climberRightEncoderChannelB")));

    m_climberExtender = new DoubleSolenoid(getInteger(robotMapProperties.getProperty("climberExtenderForwardID")),
      getInteger(robotMapProperties.getProperty("climberExtenderReverseID")));
  }

  @Override
  public void periodic() {
    if (m_encodersEnabled) {
      m_leftClimberEncoderEntry.setDouble(getLeftEncoder());
      m_rightClimberEncoderEntry.setDouble(getRightEncoder());
    }
  }

  @Override
  public int getLeftEncoder() {
    return m_leftEncoder.get();
  }

  @Override
  public int getRightEncoder() {
    return m_rightEncoder.get();
  }

  @Override
  public void climb(double leftSpeed, double rightSpeed) {
    m_leftClimbMotor.set(leftSpeed);
    m_rightClimbMotor.set(rightSpeed);
    m_logger.fine("left speed = " + leftSpeed + "right speed = " + rightSpeed);
  }

  @Override
  public void toggleClimberExtended() {
    extendClimbers(!getExtended());
  }

  // TODO: Implement climbing
  @Override
  public void extendClimbers(boolean wantsExtended) {
    m_logger.fine("extended to" + wantsExtended);
    m_climberExtender.set(wantsExtended ? m_extended : m_retracted);
  }

  private boolean getExtended() {
    if (m_shiftersEnabled) {
      return m_climberExtender.get() == m_extended ? true : false;
    } else {
      return false;
    }
  }

  @Override
  public void resetEncoders() {
    if (m_encodersEnabled) {
      m_leftEncoder.reset();
      m_rightEncoder.reset();
    }
  }

  @Override
  public Map<String, Object> getValues() {
    return null;
  }
}