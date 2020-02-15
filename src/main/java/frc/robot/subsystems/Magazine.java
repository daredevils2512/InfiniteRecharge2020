/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.Properties;
import java.util.logging.Level;
import java.util.logging.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Magazine extends SubsystemBase {
  private static Logger logger = Logger.getLogger(Magazine.class.getName());
  private final Properties properties;
  private static final String PROPERTIES_NAME = "/magazine.properties";

  private final int magazineID = -1;
  private final int queueID = -1;

  private final int ticksPerBall = -1;
  private final double arbitraryFeedForward = -1;

  private final WPI_TalonSRX magazineSpinner;
  private final WPI_TalonSRX queue;
  private final TalonSRXConfiguration magazineConfig;

  /**
   * Creates a new PowerCellManager.
   */
  public Magazine() {
    Properties defaultProperties = new Properties();
    properties = new Properties(defaultProperties);
    try {
      InputStream deployStream = new FileInputStream(Filesystem.getDeployDirectory() + PROPERTIES_NAME);
      InputStream robotStream = new FileInputStream(Filesystem.getOperatingDirectory() + PROPERTIES_NAME);
      defaultProperties.load(deployStream);
      properties.load(robotStream);
      logger.info("succesfuly loaded");
    } catch(IOException e) {
      logger.log(Level.SEVERE, "failed to load", e);
    }

    //will add properties but idk if this is the real magazine so im not doing anything about it rn
    
    magazineSpinner = new WPI_TalonSRX(this.magazineID);
    queue = new WPI_TalonSRX(this.queueID);
    magazineSpinner.setSelectedSensorPosition(0);
    magazineConfig = new TalonSRXConfiguration();
  }
  
  public void setSpeed(double speed) {
    magazineSpinner.set(ControlMode.PercentOutput, speed);
    queue.set(ControlMode.PercentOutput, speed);
  }

  public void feedBalls(int amount) {
    magazineSpinner.set(ControlMode.MotionMagic, amount * ticksPerBall, DemandType.ArbitraryFeedForward, arbitraryFeedForward);
    queue.set(1.0); // we need to know this number
  }

  public void config() {
    magazineConfig.motionCruiseVelocity = (int)SmartDashboard.getNumberArray("magazine PID array", new double[]{0, 0, 0})[0];
    magazineConfig.motionCurveStrength = (int)SmartDashboard.getNumberArray("magazine PID array", new double[]{0, 0, 0})[1];
    magazineConfig.motionAcceleration = (int)SmartDashboard.getNumberArray("magazine PID array", new double[]{0, 0, 0})[2];
    magazineSpinner.configAllSettings(magazineConfig);
  }

  @Override
  public void periodic() {

  }
}
