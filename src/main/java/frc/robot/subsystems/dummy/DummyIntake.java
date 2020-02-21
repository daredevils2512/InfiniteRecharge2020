package frc.robot.subsystems.dummy;

import java.util.Map;

import frc.robot.subsystems.interfaces.IIntake;
import frc.robot.subsystems.interfaces.IPropertySubsystem;

public class DummyIntake implements IIntake, IPropertySubsystem {

  @Override
  public void runIntake(double speed) {
  }

  @Override
  public void setMotionMagicEnabled(boolean wantsEnabled) {
  }

  @Override
  public void resetIntakeExtenderAngle() {
  }

  @Override
  public boolean getExtended() {
    return false;
  }

  @Override
  public void setExtended(boolean wantsExtended) {
  }

  @Override
  public void runExtender(double output) {
  }

  @Override
  public Map<String, Object> getValues() {
    return null;
  }

  @Override
  public void saveProperties() {
  }
  
}