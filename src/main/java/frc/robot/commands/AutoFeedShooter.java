package frc.robot.commands;

import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.interfaces.IQueue;

public class AutoFeedShooter extends CommandBase {
  private final IQueue m_queue;
  private final double m_queueSpeed;
  private final IntSupplier m_magazinePowerCellCountSupplier;

  public AutoFeedShooter(IQueue queue, double queueSpeed, IntSupplier magazinePowerCellCountSupplier) {
    m_queue = queue;
    m_queueSpeed = queueSpeed;
    m_magazinePowerCellCountSupplier = magazinePowerCellCountSupplier;
    addRequirements(queue);
  }

  @Override
  public void execute() {
    if (m_queue.hasPowerCell() || m_magazinePowerCellCountSupplier.getAsInt() > 0) {
      m_queue.run(m_queueSpeed);
    } else {
      m_queue.run(0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_queue.run(0);
  }
}