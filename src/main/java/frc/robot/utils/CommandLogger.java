package frc.robot.utils;

import java.util.logging.Level;
import java.util.logging.Logger;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Commands;

public abstract class CommandLogger extends CommandBase {
  protected final Logger m_logger;

  public CommandLogger() {
    m_logger = Logger.getLogger(Commands.class.getName());
  }

  public void setLogLevel(Level logLevel) {
    m_logger.setLevel(logLevel);
  }

  public void setLogLevel(String logLevel) {
    setLogLevel(Level.parse(logLevel.toUpperCase()));
  }
}
