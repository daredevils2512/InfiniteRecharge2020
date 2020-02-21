package frc.robot.subsystems.dummy;

import java.util.Map;

import frc.robot.subsystems.interfaces.IPropertySubsystem;
import frc.robot.subsystems.interfaces.IShooter;

public class DummyShooter implements IShooter, IPropertySubsystem {

  @Override
  public void resetHoodAngle(double angle) {
  }

  @Override
  public void setPercentOutput(double speed) {
  }

  @Override
  public void setTargetVelocity(double velocity) {
  }

  @Override
  public void stop() {
  }

  @Override
  public void setTargetAngle(double angle) {
  }

  @Override
  public double getVelocity() {
    return 0;
  }

  @Override
  public double getAngle() {
    return 0;
  }

  @Override
  public Map<String, Object> getValues() {
    return null;
  }

  @Override
  public void saveProperties() {
  }
}