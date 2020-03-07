package frc.robot.subsystems;

import java.util.Map;
import java.util.Properties;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.subsystems.interfaces.IClimber;

public class Climber extends PropertySubsystem implements IClimber {
  public static class ClimberMap {
    public int climberLeftID = -1;
    public int climberRightID = -1;

    public int shifterPortForwardID = -1; //need to move to RobotMap.proprties
    public int shifterportReverseID = -1;
    
    public int climberExtenderForwardID = 4;
    public int climberExtenderReverseID = 5;
  }

  private final WPI_TalonSRX m_leftClimbMotor;
  private final WPI_TalonSRX m_rightClimbMotor;

  private final DoubleSolenoid m_climberExtender;
  private final Value m_retracted = Value.kReverse;
  private final Value m_extended = Value.kForward;

  public Climber(Properties robotMapProperties) {
    m_leftClimbMotor = new WPI_TalonSRX(getInteger(robotMapProperties.getProperty("climberLeftID")));
    m_rightClimbMotor = new WPI_TalonSRX(getInteger(robotMapProperties.getProperty("climberRightID")));

    m_climberExtender = new DoubleSolenoid(getInteger(robotMapProperties.getProperty("climberExtendedChannel")),
      getInteger(robotMapProperties.getProperty("climberReverseChannel")));
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
    m_logger.fine("extended to" + wantsExtended);
    m_climberExtender.set(wantsExtended ? m_extended : m_retracted);
  }

  private boolean getExtended() {
    return m_climberExtender.get() == m_extended ? true : false;
  }

  @Override
  public Map<String, Object> getValues() {
    return null;
  }
}