package frc.robot.subsystems.interfaces;

import java.util.Map;

public interface IShooter extends IPropertySubsystem {
  public void resetHoodAngle(double angle);
  public void setPercentOutput(double speed);
  public void setTargetVelocity(double velocity);
  public void stop();
  void setHoodSpeed(double speed);
  public void setTargetAngle(double angle);
  public double getVelocity();
  public double getAngle();
  public Map<String, Object> getValues();
}
