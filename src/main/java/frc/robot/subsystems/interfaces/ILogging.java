package frc.robot.subsystems.interfaces;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ILogging extends Subsystem {
  default void setLogLevel(String logLevel) { }
}