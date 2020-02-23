package frc.robot.subsystems.dummy;

import java.util.Map;

import frc.robot.subsystems.interfaces.IMagazine;
import frc.robot.subsystems.interfaces.IPropertySubsystem;

public class DummyMagazine implements IMagazine, IPropertySubsystem {

  @Override
  public boolean getPowerCellDetected() {
    return false;
  }

  @Override
  public void updatePowerCellCount() {
  }

  @Override
  public boolean getDirectionReversed() {
    return false;
  }

  @Override
  public void setSpeed(double speed) {
  }

  @Override
  public void feedBalls(int amount) {
  }

  @Override
  public Map<String, Object> getValues() {
    return null;
  }

  @Override
  public void saveProperties() {
  }
}