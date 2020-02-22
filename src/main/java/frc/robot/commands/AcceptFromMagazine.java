package frc.robot.commands;

import java.util.function.IntSupplier;

import frc.robot.subsystems.interfaces.IQueue;

public class AcceptFromMagazine extends RunQueueCommand {
  public AcceptFromMagazine(IQueue queue, double queueSpeed, IntSupplier magazinePowerCellCountSupplier) {
    super(queue, queueSpeed, magazinePowerCellCountSupplier);
  }

  @Override
  protected boolean shouldRunQueue() {
    return !m_queue.hasPowerCell()
      && m_magazinePowerCellCountSupplier.getAsInt() > 0;
  }
}
