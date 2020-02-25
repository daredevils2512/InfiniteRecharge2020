package frc.robot.subsystems.interfaces;

public interface ITurret extends IPropertySubsystem {
  public double getAngle();
  public void resetEncoder();
  public void setSpeed(double speed);
  public void runPosition(double degrees);
  public double wrapDegrees(double degrees);
  public void setTargetAngle(double angle);
}
