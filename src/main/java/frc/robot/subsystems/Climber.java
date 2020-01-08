package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private final int m_leftClimberMasterID = -1;
  private final int m_rightClimberMasterID = -1;
  private final int m_leftHorizontalMasterID = -1;
  private final int m_rightHorizontalMasterID = -1;

  private final WPI_TalonSRX m_leftClimberMaster;
  private final WPI_TalonSRX m_rightClimberMaster;

  private final WPI_TalonSRX m_leftHorizontalMaster;
  private final WPI_TalonSRX m_rightHorizontalMaster;

  public Climber() {
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


  public void climbLeft(Drivetrain drivetrain, double speed) {
    drivetrain.driveLeft(speed);
  }

  public void climbRight(Drivetrain drivetrain, double speed) {
    drivetrain.driveRight(speed);
  }


}