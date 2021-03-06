package frc.robot.subsystems;

import java.util.Map;
import java.util.Properties;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.subsystems.interfaces.IClimber;

public class Climber extends PropertySubsystem implements IClimber {
  public static class ClimberMap {
    public int climberLeftID = -1;
    public int climberRightID = -1;
  }

  private final WPI_TalonSRX m_leftClimbMotor;
  private final WPI_TalonSRX m_rightClimbMotor;

  public Climber(Properties robotMapProperties) {
    m_leftClimbMotor = new WPI_TalonSRX(getInteger(robotMapProperties.getProperty("climberLeftID")));
    m_rightClimbMotor = new WPI_TalonSRX(getInteger(robotMapProperties.getProperty("climberRightID")));
  }

  @Override
  public void climb(double leftSpeed, double rightSpeed) {
    m_leftClimbMotor.set(leftSpeed);
    m_rightClimbMotor.set(rightSpeed);
    m_logger.fine("left speed = " + leftSpeed + "right speed = " + rightSpeed);
  }

  // TODO: Implement climbing
  @Override
  public void climbLeft(Drivetrain drivetrain, double speed) {

  }

  @Override
  public void climbRight(Drivetrain drivetrain, double speed) {

  }

  @Override
  public Map<String, Object> getValues() {
    return null;
  }
}