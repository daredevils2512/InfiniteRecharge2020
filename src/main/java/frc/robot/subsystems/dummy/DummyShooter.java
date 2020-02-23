package frc.robot.subsystems.dummy;

import java.util.Map;

import frc.robot.subsystems.interfaces.IPropertySubsystem;
import frc.robot.subsystems.interfaces.IShooter;

public class DummyShooter implements IShooter, IPropertySubsystem {

  @Override
  public void resetHoodAngle(double angle) {
  }

  @Override
  public void setShooterPercentOutput(double speed) {
  }

  @Override
  public void setShooterVelocity(double velocity) {
  }

  @Override
  public double getShooterVelocity() {
    return 0;
  }

  @Override
  public boolean shooterAtSetpoint(double tolerance) {
    return false;
  }

  @Override
  public void setHoodAngle(double angle) {
  }

  @Override
  public double getHoodAngle() {
    return 0;
  }

  @Override
  public boolean hoodAtSetpoint(double tolerance) {
    return false;
  }

  @Override
  public Map<String, Object> getValues() {
    return null;
  }

  @Override
  public void saveProperties() {
  }
}