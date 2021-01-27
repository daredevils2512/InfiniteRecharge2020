package frc.robot.commands;

import frc.robot.subsystems.interfaces.IQueue;
import frc.robot.utils.CommandLogger;

public abstract class RunQueueCommand extends CommandLogger {
  protected final IQueue m_queue;
  protected final double m_queueSpeed;

  public RunQueueCommand(IQueue queue, double queueSpeed) {
    m_queue = queue;
    m_queueSpeed = queueSpeed;
    addRequirements(queue);
  }

  @Override
  public void execute() {
    m_queue.run(shouldRunQueue() ? m_queueSpeed : 0);
  }

  @Override
  public void end(boolean interrupted) {
    m_logger.fine("interrupted");
    m_queue.run(0);
  }

  protected abstract boolean shouldRunQueue();
}
