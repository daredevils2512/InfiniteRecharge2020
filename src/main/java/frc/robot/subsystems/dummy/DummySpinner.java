package frc.robot.subsystems.dummy;

import frc.robot.sensors.ColorSensor.ColorDetect;
import frc.robot.subsystems.interfaces.ISpinner;

public class DummySpinner implements ISpinner {

  @Override
  public ColorDetect getCurrentColor() {
    return null;
  }

  @Override
  public void setExtended(boolean wantsExtended) {
  }

  @Override
  public void run(double speed) {
  }
  
}