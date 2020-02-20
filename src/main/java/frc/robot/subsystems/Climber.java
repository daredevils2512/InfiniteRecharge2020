package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Climber extends PropertySubsystem {
  public static class ClimberMap {
    public int climberLeftID = -1;
    public int climberRightID = -1;
  }

  private final WPI_TalonSRX m_leftClimbMotor;
  private final WPI_TalonSRX m_rightClimbMotor;

  public Climber(ClimberMap climberMap) {
    super(Climber.class);

    m_leftClimbMotor = new WPI_TalonSRX(climberMap.climberLeftID);
    m_rightClimbMotor = new WPI_TalonSRX(climberMap.climberRightID);
  }

  public void climb(double leftSpeed, double rightSpeed) {
    m_leftClimbMotor.set(leftSpeed);
    m_rightClimbMotor.set(rightSpeed);
  }

  // TODO: Implement climbing
  public void climbLeft(Drivetrain drivetrain, double speed) {

  }

  public void climbRight(Drivetrain drivetrain, double speed) {

  }

  @Override
  protected Map<String, Object> getValues() {
    return null;
  }
}