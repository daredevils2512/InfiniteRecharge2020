package frc.robot.subsystems.dummy;

import frc.robot.subsystems.interfaces.IHexagonPosition;

public class DummyHexagonPosition implements IHexagonPosition {

  @Override
  public void updatePosition() {
  }
  
  @Override
  public boolean canShoot() {
    return false;
  }
}