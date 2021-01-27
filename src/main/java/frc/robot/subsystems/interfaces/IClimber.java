package frc.robot.subsystems.interfaces;

import frc.robot.subsystems.Drivetrain;

public interface IClimber extends IPropertySubsystem {
  public void climb(double leftSpeed, double rightSpeed);
  public void climbLeft(Drivetrain drivetrain, double speed);
  public void climbRight(Drivetrain drivetrain, double speed);
}