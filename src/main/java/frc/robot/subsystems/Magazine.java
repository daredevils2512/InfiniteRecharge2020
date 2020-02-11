/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.sensors.Photoeye;

public class Magazine extends SubsystemBase {
  private final NetworkTable m_networkTable;
  
  private final int m_photoeye1ID = -1;
  private final int m_photoeye2ID = -1;
  private final Photoeye m_photoeye1;
  private final Photoeye m_photoeye2;

  private final int magazineID = -1;

  private final int ticksPerBall = -1;
  private final double arbitraryFeedForward = -1;

  private final WPI_TalonSRX magazineSpinner;
  private final TalonSRXConfiguration magazineConfig;
  
  //these are for the ball counter
  private int ballCount;
  private boolean ballIn;
  private boolean ballOut;
  private boolean invalidBallCount;
  private boolean countInvert;

  /**
   * Creates a new PowerCellManager.
   */
  public Magazine() {
    magazineSpinner = new WPI_TalonSRX(this.magazineID);
    magazineSpinner.setSelectedSensorPosition(0);
    magazineConfig = new TalonSRXConfiguration();

    m_photoeye1 = new Photoeye(m_photoeye1ID);
    m_photoeye2 = new Photoeye(m_photoeye2ID);

    m_networkTable = NetworkTableInstance.getDefault().getTable(getName());
    
    ballCount = 0;
    ballIn = false;
    ballOut = false;
    invalidBallCount = false;
    countInvert = false;
  }
  
  public void setSpeed(double speed) {
    magazineSpinner.set(ControlMode.PercentOutput, speed);
    countInvert = speed >= 0 ? false : true;
  }

  public void feedBalls(int amount) {
    magazineSpinner.set(ControlMode.MotionMagic, amount * ticksPerBall, DemandType.ArbitraryFeedForward, arbitraryFeedForward);
    countInvert = amount * ticksPerBall >= 0 ? false : true;
  }

  public void config() {
    magazineConfig.motionCruiseVelocity = (int)SmartDashboard.getNumberArray("magazine PID array", new double[]{0, 0, 0})[0];
    magazineConfig.motionCurveStrength = (int)SmartDashboard.getNumberArray("magazine PID array", new double[]{0, 0, 0})[1];
    magazineConfig.motionAcceleration = (int)SmartDashboard.getNumberArray("magazine PID array", new double[]{0, 0, 0})[2];
    magazineSpinner.configAllSettings(magazineConfig);
  }
  
  public boolean getInBall() {
    return !m_photoeye1.get();
  }

  public boolean getOutBall() {
    return !m_photoeye2.get();
  }

  public void setBallsInMag(int set) {
    ballCount = set;
  }

  public void resetBallCount() {
    setBallsInMag(0);
  }

  public boolean getInvalidBallCount() {
    return invalidBallCount;
  }
  
  //might be temporary
  public int countBall() {  
    if (!countInvert) {
      if (getInBall()) {
        ballIn = true;
      } else if (!getInBall() && ballIn) {
        ballIn = false;
        ballCount += 1;
      }

      if (getOutBall()) {
        ballOut = true;
      } else if (!getOutBall() && ballOut) {
        ballOut = false;
        ballCount -= 1;
      }
    } else {
      if (getInBall()) {
        ballIn = true;
      } else if (!getInBall() && ballIn) {
        ballIn = false;
        ballCount -= 1;
      }
    }

    if (ballCount < 0 || ballCount > 3) {
      invalidBallCount = true;
    } else {
      invalidBallCount = false;
    }

    return MathUtil.clamp(ballCount, 0, 3);
  }

  public void updateBall() {
    countBall();
    SmartDashboard.putNumber("balls in magazine", countBall());
    SmartDashboard.putBoolean("invalid ball count", getInvalidBallCount());
    if (getInvalidBallCount()) {
      System.out.println("INVALID BALL COUNT");
    }
  }

  @Override
  public void periodic() {
    updateBall();
  }
}
