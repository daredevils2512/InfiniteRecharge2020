package frc.robot.subsystems.interfaces;

import java.util.Map;

public interface IMagazine extends IPropertySubsystem {
  public boolean getPowerCellDetected();
  public void updatePowerCellCount();
  public boolean getDirectionReversed();

  public void setSpeed(double speed);
  public void feedBalls(int amount);

  public Map<String, Object> getValues();
}
