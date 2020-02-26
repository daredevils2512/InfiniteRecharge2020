package frc.robot.commands;

import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.interfaces.IQueue;

public abstract class RunQueueCommand extends CommandBase {
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
    m_queue.run(0);
  }

  protected abstract boolean shouldRunQueue();
}
