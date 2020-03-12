package frc.robot.subsystems.interfaces;

import java.util.Map;

import frc.robot.sensors.IDigitalInput;

public interface IMagazine extends IPropertySubsystem {
  public boolean getPowerCellDetected();
  public boolean getDirectionReversed();

  public void setSpeed(double speed);
  public void feedBalls(int amount);
  public int getCount();

  public Map<String, Object> getValues();
  public IDigitalInput getPhotoEye();
}
