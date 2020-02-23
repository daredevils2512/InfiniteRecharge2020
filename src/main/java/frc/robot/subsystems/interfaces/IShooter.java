package frc.robot.subsystems.interfaces;

public interface IShooter extends IPropertySubsystem {
  public void setShooterPercentOutput(double speed);
  public void setShooterVelocity(double velocity);
  public double getShooterVelocity();
  public boolean shooterAtSetpoint(double tolerance);
  public void resetHoodAngle(double angle);
  public void setHoodAngle(double angle);
  public double getHoodAngle();
  public boolean hoodAtSetpoint(double tolerance);
}
