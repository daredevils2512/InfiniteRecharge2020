package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;

import frc.robot.subsystems.interfaces.IMagazine;
import frc.robot.utils.CommandLogger;

public class RefillQueue extends CommandLogger {
  private final IMagazine m_magazine;
  private final double m_magazineSpeed;
  private final IntSupplier m_magazinePowerCellCountSupplier;
  private final BooleanSupplier m_queueHasPowerCellSupplier;

  public RefillQueue(IMagazine magazine, double magazineSpeed, IntSupplier magazinePowerCellCountSupplier, BooleanSupplier queueHasPowerCellSupplier) {
    m_magazine = magazine;
    m_magazineSpeed = magazineSpeed;
    m_magazinePowerCellCountSupplier = magazinePowerCellCountSupplier;
    m_queueHasPowerCellSupplier = queueHasPowerCellSupplier;
    addRequirements(m_magazine);
  }

  @Override
  public void execute() {
    m_logger.fine("refilling queue");
    m_magazine.setSpeed(m_magazineSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    m_magazine.setSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return m_magazinePowerCellCountSupplier.getAsInt() < 1 || m_queueHasPowerCellSupplier.getAsBoolean();
  }
}
