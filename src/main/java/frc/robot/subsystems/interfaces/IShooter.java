package frc.robot.subsystems.interfaces;

public interface IShooter extends IPropertySubsystem {
  public void resetHoodAngle(double angle);
  public void setPercentOutput(double speed);
  public void setTargetVelocity(double velocity);
  public void stop();
  void setHoodSpeed(double speed);
  public void setTargetAngle(double angle);
  public double getVelocity();
  public double getAngle();
  public double getCalculatedVelocity();
  public boolean isAtSpeed();
}
