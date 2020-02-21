package frc.robot.subsystems.interfaces;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface IMagazine extends Subsystem, IPropertySubsystem {
  public boolean getPowerCellDetected();
  public void updatePowerCellCount();
  public boolean getDirectionReversed();

  public void setSpeed(double speed);
  public void feedBalls(int amount);

  public Map<String, Object> getValues();
}
