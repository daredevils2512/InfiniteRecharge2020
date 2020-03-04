package frc.robot.commands;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.interfaces.IQueue;
import frc.robot.utils.CommandLogger;

public class FeedShooter extends CommandLogger {
  private IQueue m_queue;
  private DoubleSupplier m_queueSpeedSupplier;

  public FeedShooter(IQueue queue, DoubleSupplier queueSpeedSupplier) {
    m_queue = queue;
    m_queueSpeedSupplier = queueSpeedSupplier;
    addRequirements(queue);
  }

  @Override
  public void execute() {
    m_queue.run(m_queueSpeedSupplier.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    m_queue.run(0);
  }

  @Override
  public boolean isFinished() {
    return !m_queue.hasPowerCell();
  }
}