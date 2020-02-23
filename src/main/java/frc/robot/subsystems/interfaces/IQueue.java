package frc.robot.subsystems.interfaces;

import java.util.Map;

import frc.robot.sensors.IDigitalInput;
import frc.robot.sensors.PhotoEye;

public interface IQueue extends IPropertySubsystem {
  public void run(double speed);
  public boolean getDirectionReversed();
  public boolean hasPowerCell();
  public Map<String, Object> getValues();
  public IDigitalInput getPhotoEye();
}
