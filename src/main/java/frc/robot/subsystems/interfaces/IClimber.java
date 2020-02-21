package frc.robot.subsystems.interfaces;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Drivetrain;

public interface IClimber extends Subsystem, IPropertySubsystem {
  public void climb(double leftSpeed, double rightSpeed);
  public void climberMoveHorizontal(double speed);
  public void climbLeft(Drivetrain drivetrain, double speed);
  public void climbRight(Drivetrain drivetrain, double speed);
  public Map<String, Object> getValues();
}