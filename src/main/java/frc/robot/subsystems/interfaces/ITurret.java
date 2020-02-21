package frc.robot.subsystems.interfaces;

import java.util.Map;

public interface ITurret {
  public double getAngle();
  public void resetEncoder();
  public void setSpeed(double speed);
  public void runPosition(double degrees);
  public double wrapDegrees(double degrees);
  public void setTargetAngle(double angle);
  public Map<String, Object> getValues();
}
