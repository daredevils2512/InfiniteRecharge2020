package frc.robot.subsystems.interfaces;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface IShooter extends Subsystem, IPropertySubsystem {
  public void resetHoodAngle(double angle);
  public void setPercentOutput(double speed);
  public void setTargetVelocity(double velocity);
  public void stop();
  public void setTargetAngle(double angle);
  public double getVelocity();
  public double getAngle();
  public Map<String, Object> getValues();
}
