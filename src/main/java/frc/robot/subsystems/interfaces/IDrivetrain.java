/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.interfaces;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
/**
 * The drivetrain is a 6 wheel west coast differential drivetrain with two-gear
 * transmission. It consists of four {@link TalonFX} motor controllers for
 * driving (two per side), a {@link DoubleSolenoid} for shifting, two 256PPR
 * optical encoders (one per side) mounted to the output of the gearbox for
 * distance calculation, and a {@link PigeonIMU} for heading calculation.
 */
public interface IDrivetrain extends IPropertySubsystem {
  public double getMaxSpeed();
  public double getMaxAngularSpeed();
  public double getMaxAcceleration();

  public boolean getDrivingInverted();
  public void setDrivingInverted(boolean wantsInverted);

  public double getLeftDistance();
  public double getRightDistance();
  public double getLeftVelocity();
  public double getRightVelocity();

  public DifferentialDriveKinematics getKinematics();

  public double getHeading();

  public void setLowGear(boolean wantsLowGear);
  public boolean getLowGear();

  public Pose2d getPose();
  public void resetPose();
  public void resetPose(Translation2d translation);
  public void resetPose(Rotation2d rotation);
  public void resetPose(Pose2d pose);

  public void simpleArcadeDrive(double move, double turn);
  public void voltageTank(double left, double right);
  public void velocityArcadeDrive(double velocity, double angularVelocity);
  public void setWheelSpeeds(double left, double right);
}