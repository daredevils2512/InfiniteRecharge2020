package frc.robot.subsystems;

import java.util.logging.Level;
import java.util.logging.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase; 
import frc.robot.subsystems.interfaces.ILogging;

public abstract class LoggingSubsystem extends SubsystemBase implements ILogging {
  protected final Logger m_logger;

  public LoggingSubsystem() {
    m_logger = Logger.getLogger(getClass().getName());
  }

  public void setLogLevel(Level logLevel) {
    m_logger.setLevel(logLevel);
  }

  public void setLogLevel(String logLevel) {
    setLogLevel(Level.parse(logLevel.toUpperCase()));
  }
}
