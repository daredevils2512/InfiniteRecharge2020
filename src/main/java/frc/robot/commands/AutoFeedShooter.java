package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Queue;

public class AutoFeedShooter extends CommandBase {
  private final Queue m_queue;
  private final DoubleSupplier m_queueSpeedSupplier;
  private final IntSupplier m_magazinePowerCellCountSupplier;

  public AutoFeedShooter(Queue queue, DoubleSupplier queueSpeedSupplier, IntSupplier magazinePowerCellCountSupplier) {
    m_queue = queue;
    m_queueSpeedSupplier = queueSpeedSupplier;
    m_magazinePowerCellCountSupplier = magazinePowerCellCountSupplier;
    addRequirements(queue);
  }

  @Override
  public void execute() {
    if (m_queue.hasPowerCell() || m_magazinePowerCellCountSupplier.getAsInt() > 0) {
      m_queue.run(m_queueSpeedSupplier.getAsDouble());
    } else {
      m_queue.run(0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_queue.run(0);
  }
}