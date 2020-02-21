package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.interfaces.IIntake;
import frc.robot.subsystems.interfaces.IMagazine;

public class RunIntake extends CommandBase {
  private final IIntake m_intake;
  private final IMagazine m_magazine;
  private final double m_speed;

  public RunIntake(IIntake intake, IMagazine magazine, double speed) {
    m_intake = intake;
    m_magazine = magazine;
    m_speed = speed;
    addRequirements(intake, magazine);
  }

  @Override
  public void initialize() {
    m_intake.setExtended(true);
  }

  @Override
  public void execute() {
    m_magazine.setSpeed(m_speed);
  }

  @Override
  public void end(boolean interrupted) {
    m_magazine.setSpeed(0);
  }
}