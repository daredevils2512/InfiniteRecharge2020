package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;

public class RunIntake extends CommandBase {
  private final Intake m_intake;
  private final Magazine m_magazine;
  private final double m_speed;

  public RunIntake(Intake intake, Magazine magazine, double speed) {
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