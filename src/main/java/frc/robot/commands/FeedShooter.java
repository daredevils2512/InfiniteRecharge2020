package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Queue;

public class FeedShooter extends CommandBase {
  private Queue m_queue;
  private DoubleSupplier m_queueSpeedSupplier;

  public FeedShooter(Queue queue, DoubleSupplier queueSpeedSupplier) {
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