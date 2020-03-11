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

  private final DoubleSolenoid m_climbShifter;
  private final Value m_climbing = Value.kForward;
  private final Value m_driving = Value.kReverse;

  private final NetworkTable m_networktable;
  private final NetworkTableEntry m_leftClimberEncoderEntry;
  private final NetworkTableEntry m_rightClimberEncoderEntry;
  private final NetworkTableEntry m_resetEncoderEntry;


  public Climber(Properties robotMapProperties) {
    m_shiftersEnabled = getBoolean(m_properties.getProperty("shiftersEnabled"));
    m_encodersEnabled = getBoolean(m_properties.getProperty("encodersEnabled"));

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
    m_climbShifter = new DoubleSolenoid(getInteger(robotMapProperties.getProperty("shifterPortForwardID")),
        getInteger(robotMapProperties.getProperty("shifterPortReverseID")));
    
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
    m_leftClimbMotor.set(speed);
  }

  public void extendRightClimber(double speed) {
    m_rightClimbMotor.set(speed);
  }

  @Override
  public void toggleClimberExtended() {
    raiseClimbers(!getExtended());
  }

  @Override
  public void raiseClimbers(boolean wantsExtended) {
    m_logger.info("extended to" + wantsExtended);
    m_climberExtender.set(wantsExtended ? m_extended : m_retracted);
  }

  public boolean getExtended() {
    return m_climberExtender.get() == m_extended;
  }

  public boolean getShifted() {
    return m_climbShifter.get() == m_climbing;
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