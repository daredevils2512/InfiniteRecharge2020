package frc.robot.subsystems.dummy;

import java.util.Map;
import frc.robot.subsystems.interfaces.ITurret;

public class DummyTurret implements ITurret {

  @Override
  public double getAngle() {
    return 0;
  }

  @Override
  public void resetEncoder() {
  }

  @Override
  public void setSpeed(double speed) {
  }

  @Override
  public void runPosition(double degrees) {
  }

  @Override
  public double wrapDegrees(double degrees) {
    return 0;
  }

  @Override
  public void setTargetAngle(double angle) {
  }

  @Override
  public Map<String, Object> getValues() {
    return null;
  }

  @Override
  public void saveProperties() {
  } 
}