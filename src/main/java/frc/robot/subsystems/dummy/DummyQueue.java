package frc.robot.subsystems.dummy;

import java.util.Map;

import frc.robot.subsystems.interfaces.IPropertySubsystem;
import frc.robot.subsystems.interfaces.IQueue;

public class DummyQueue implements IQueue, IPropertySubsystem {
  @Override
  public void onPowerCellInMagazine(Runnable runnable) {
    
  }

  @Override
  public void onPowerCellInShooter(Runnable runnable) {
    
  }

  @Override
  public void onPowerCellOutMagazine(Runnable runnable) {
    
  }

  @Override
  public void onPowerCellOutShooter(Runnable runnable) {
    
  }

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
}