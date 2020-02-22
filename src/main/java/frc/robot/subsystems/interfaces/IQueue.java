package frc.robot.subsystems.interfaces;

import java.util.Map;

public interface IQueue extends IPropertySubsystem {
  public void run(double speed);
  public boolean getDirectionReversed();
  public boolean hasPowerCell();

}
