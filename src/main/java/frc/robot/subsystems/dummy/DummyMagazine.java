package frc.robot.subsystems.dummy;

import java.util.Map;

import frc.robot.sensors.DummyDigitalInput;
import frc.robot.sensors.IDigitalInput;
import frc.robot.subsystems.interfaces.IMagazine;

public class DummyMagazine implements IMagazine {

  @Override
  public boolean getPowerCellDetected() {
    return false;
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

  @Override
  public IDigitalInput getPhotoEye() {
    return new DummyDigitalInput();
  }

  @Override
  public int getCount() {
    return 0;
  }
}