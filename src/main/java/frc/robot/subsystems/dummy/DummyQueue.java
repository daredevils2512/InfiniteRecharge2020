package frc.robot.subsystems.dummy;

import java.util.Map;

import frc.robot.sensors.DummyDigitalInput;
import frc.robot.sensors.IDigitalInput;
import frc.robot.subsystems.interfaces.IQueue;

public class DummyQueue implements IQueue {

  @Override
  public void run(double speed) {
  }

  @Override
  public boolean getDirectionReversed() {
    return false;
  }

  @Override
  public boolean hasPowerCell() {
    return false;
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
}