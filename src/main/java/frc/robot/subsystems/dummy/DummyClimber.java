package frc.robot.subsystems.dummy;

import java.util.Map;

import frc.robot.subsystems.interfaces.IClimber;

public class DummyClimber implements IClimber {

  @Override
  public void extendClimbers(double leftSpeed, double rightSpeed) {
  }

  @Override
  public Map<String, Object> getValues() {
    return null;
  }

  @Override
  public void saveProperties() {
  }

  @Override
  public void raiseClimbers(boolean wantsExtended) {
  }

  @Override
  public void toggleClimberExtended() {
  }

  @Override
  public void extendLeftClimber(double speed) {
  }

  @Override
  public void extendRightClimber(double speed) {
  }
  
  public void resetEncoders() {
  }

  @Override
  public int getLeftEncoder() {
    return 0;
  }

  @Override
  public int getRightEncoder() {
    return 0;
  }
}