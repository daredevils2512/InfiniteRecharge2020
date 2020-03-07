package frc.robot.subsystems.dummy;

import java.util.Map;

import frc.robot.subsystems.interfaces.IClimber;

public class DummyClimber implements IClimber {

  @Override
  public void climb(double leftSpeed, double rightSpeed) {
  }

  @Override
  public Map<String, Object> getValues() {
    return null;
  }

  @Override
  public void saveProperties() {
  }

  @Override
  public void extendClimbers(boolean wantsExtended) {
  }

  @Override
  public void toggleClimberExtended() {
  }

  @Override
  public void resetEncoders() {
  }
}