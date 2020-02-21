package frc.robot.subsystems.interfaces;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface IQueue extends Subsystem, IPropertySubsystem {
  public void run(double speed);
  public boolean getDirectionReversed();
  public boolean hasPowerCell();
  public Map<String, Object> getValues();
}
