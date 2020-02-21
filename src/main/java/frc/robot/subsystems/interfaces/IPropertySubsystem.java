package frc.robot.subsystems.interfaces;

public interface IPropertySubsystem extends ILogging {
  default void saveProperties() { }
}