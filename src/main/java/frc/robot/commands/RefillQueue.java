package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.interfaces.IMagazine;

public class RefillQueue extends CommandBase {
  private final IMagazine m_magazine;
  private final double m_magazineSpeed;
  private final BooleanSupplier m_powerCellQueued;

  public RefillQueue(IMagazine magazine, double magazineSpeed, BooleanSupplier powerCellQueued) {
    m_magazine = magazine;
    m_magazineSpeed = magazineSpeed;
    m_powerCellQueued = powerCellQueued;
    addRequirements(m_magazine);
  }

  @Override
  public void execute() {
    m_magazine.setSpeed(m_magazineSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    m_magazine.setSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return m_magazine.getPowerCellCount() < 1 || m_powerCellQueued.getAsBoolean();
  }
}
