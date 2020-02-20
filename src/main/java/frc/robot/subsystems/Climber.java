package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Climber extends PropertySubsystem {
  // dont have numbers for these
  private final int m_leftClimberMasterID;
  private final int m_rightClimberMasterID;
  private final int m_leftHorizontalMasterID;
  private final int m_rightHorizontalMasterID;

  private final WPI_TalonSRX m_leftClimberMaster;
  private final WPI_TalonSRX m_rightClimberMaster;

  private final WPI_TalonSRX m_leftHorizontalMaster;
  private final WPI_TalonSRX m_rightHorizontalMaster;

  public Climber() {
    super(Climber.class.getName());

    m_leftClimberMasterID = Integer.parseInt(properties.getProperty("leftClimberMasterID"));
    m_rightClimberMasterID = Integer.parseInt(properties.getProperty("rightClimberMasterID"));
    m_leftHorizontalMasterID = Integer.parseInt(properties.getProperty("leftHorizontalMasterID"));
    m_rightHorizontalMasterID = Integer.parseInt(properties.getProperty("rightHorizontalMasterID"));

    m_leftClimberMaster = new WPI_TalonSRX(m_leftClimberMasterID);
    m_rightClimberMaster = new WPI_TalonSRX(m_rightClimberMasterID);

    m_leftHorizontalMaster = new WPI_TalonSRX(m_leftHorizontalMasterID);
    m_rightHorizontalMaster = new WPI_TalonSRX(m_rightHorizontalMasterID);
  }

  public void climb(double leftSpeed, double rightSpeed) {
    m_leftClimberMaster.set(leftSpeed);
    m_rightClimberMaster.set(rightSpeed);
  }

  public void climberMoveHorizontal(double speed) {
    m_leftHorizontalMaster.set(speed);
    m_rightHorizontalMaster.set(speed);
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