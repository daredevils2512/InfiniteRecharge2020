package frc.robot.subsystems.interfaces;

import java.util.Map;

public interface IPropertySubsystem extends ILogging {
  default void saveProperties() { }
  public Map<String, Object> getValues();

}