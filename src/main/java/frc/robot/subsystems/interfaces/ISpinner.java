package frc.robot.subsystems.interfaces;

import frc.robot.sensors.ColorSensor.ColorDetect;

public interface ISpinner extends ILogging {
  public ColorDetect getCurrentColor();
  public void setExtended(boolean wantsExtended);
  public void run(double speed);
}
