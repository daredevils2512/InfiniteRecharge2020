package frc.robot.subsystems.interfaces;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ITurret extends Subsystem, IPropertySubsystem {
  public double getAngle();
  public void resetEncoder();
  public void setSpeed(double speed);
  public void runPosition(double degrees);
  public double wrapDegrees(double degrees);
  public void setTargetAngle(double angle);
  public Map<String, Object> getValues();
}
