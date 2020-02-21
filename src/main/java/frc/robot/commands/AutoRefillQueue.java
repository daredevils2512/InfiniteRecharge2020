package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.interfaces.IMagazine;

public class AutoRefillQueue extends CommandBase {
  private final IMagazine m_magazine;
  private final double m_magazineSpeed;
  private final BooleanSupplier m_powerCellQueued;

  public AutoRefillQueue(IMagazine magazine, double magazineSpeed, BooleanSupplier powerCellQueued) {
    m_magazine = magazine;
    m_magazineSpeed = magazineSpeed;
    m_powerCellQueued = powerCellQueued;
    addRequirements(magazine);
  }

  @Override
  public void execute() {
    if (m_powerCellQueued.getAsBoolean())
      m_magazine.setSpeed(0);
    else
      m_magazine.setSpeed(m_magazineSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    m_magazine.setSpeed(0);
  }
}
