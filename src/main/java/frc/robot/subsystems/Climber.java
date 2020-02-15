package frc.robot.subsystems;

import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.Properties;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  //dont have numbers for these
  private final int m_leftClimberMasterID;
  private final int m_rightClimberMasterID;
  private final int m_leftHorizontalMasterID;
  private final int m_rightHorizontalMasterID;

  private final WPI_TalonSRX m_leftClimberMaster;
  private final WPI_TalonSRX m_rightClimberMaster;

  private final WPI_TalonSRX m_leftHorizontalMaster;
  private final WPI_TalonSRX m_rightHorizontalMaster;

  private final Properties properties;
  private static final String PROPERTIES_NAME = "/climber.properties";

  public Climber() {
    Properties defaultProperties = new Properties();
    properties = new Properties(defaultProperties);
    try {
      InputStream deployStream = new FileInputStream(Filesystem.getDeployDirectory() + PROPERTIES_NAME);
      InputStream robotStream = new FileInputStream(Filesystem.getOperatingDirectory() + PROPERTIES_NAME);
      defaultProperties.load(deployStream);
      properties.load(robotStream);
    } catch(IOException e) {
      e.printStackTrace();
    }

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


}