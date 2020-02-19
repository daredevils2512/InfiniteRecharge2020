package frc.robot.subsystems;

import java.util.Map;
import java.util.Properties;
import java.util.logging.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.PropertyFiles;

public class Climber extends PropertySubsystem {
  // dont have numbers for these
  private final int m_leftClimberMasterID;
  private final int m_rightClimberMasterID;
  private final int m_leftHorizontalMasterID;
  private final int m_rightHorizontalMasterID;

  private final WPI_TalonSRX m_leftClimberMaster;
  private final WPI_TalonSRX m_rightClimberMaster;
  private final boolean m_useVerticalClimb;

  private final WPI_TalonSRX m_leftHorizontalMaster;
  private final WPI_TalonSRX m_rightHorizontalMaster;
  private final boolean m_useHorizontalClimb;

  public Climber() {
    super(Climber.class.getSimpleName());

    m_leftClimberMasterID = Integer.parseInt(properties.getProperty("leftClimberMasterID"));
    m_rightClimberMasterID = Integer.parseInt(properties.getProperty("rightClimberMasterID"));
    m_leftHorizontalMasterID = Integer.parseInt(properties.getProperty("leftHorizontalMasterID"));
    m_rightHorizontalMasterID = Integer.parseInt(properties.getProperty("rightHorizontalMasterID"));
    m_useVerticalClimb = Boolean.parseBoolean(properties.getProperty("useVerticalClimb"));
    m_useHorizontalClimb = Boolean.parseBoolean(properties.getProperty("useHorizontalClimb"));

    if (m_useVerticalClimb) {
      m_leftClimberMaster = new WPI_TalonSRX(m_leftClimberMasterID);
      m_rightClimberMaster = new WPI_TalonSRX(m_rightClimberMasterID);
    } else {
      m_leftClimberMaster = null;
      m_rightClimberMaster = null;
    }

    if (m_useHorizontalClimb) {
      m_leftHorizontalMaster = new WPI_TalonSRX(m_leftHorizontalMasterID);
      m_rightHorizontalMaster = new WPI_TalonSRX(m_rightHorizontalMasterID);
    } else {
      m_leftHorizontalMaster = null;
      m_rightHorizontalMaster = null;
    }
  }

  public void climb(double leftSpeed, double rightSpeed) {
    if (m_useVerticalClimb) {
      m_leftClimberMaster.set(leftSpeed);
      m_rightClimberMaster.set(rightSpeed);
    }
  }

  public void climberMoveHorizontal(double speed) {
    if (m_useHorizontalClimb) {
      m_leftHorizontalMaster.set(speed);
      m_rightHorizontalMaster.set(speed);
    }
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