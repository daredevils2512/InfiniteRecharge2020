package frc.robot.subsystems.interfaces;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.sensors.ColorSensor.ColorDetect;

public interface ISpinner extends Subsystem {
  public ColorDetect getCurrentColor();
  public void setExtended(boolean wantsExtended);
  public void run(double speed);
}
