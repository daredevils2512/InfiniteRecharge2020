package frc.robot.subsystems.dummy;

import java.util.Map;

import frc.robot.subsystems.interfaces.IPropertySubsystem;
import frc.robot.subsystems.interfaces.IQueue;

public class DummyQueue implements IQueue, IPropertySubsystem {

  @Override
  public void run(double speed) {
  }

  @Override
  public void run(double speed, boolean wantsClosed) {
  }

  @Override
  public boolean getClosed() {
    return false;
  }

  @Override
  public void setClosed(boolean wantsClosed) {
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
  
}