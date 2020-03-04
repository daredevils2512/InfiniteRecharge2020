package frc.robot.commands;

import frc.robot.subsystems.interfaces.IIntake;
import frc.robot.utils.CommandLogger;

public class RunIntake extends CommandLogger {
  private final IIntake m_intake;
  private final double m_speed;

  public RunIntake(IIntake intake, double speed) {
    m_intake = intake;
    m_speed = speed;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    m_intake.setMotionMagicEnabled(false);
  }

  @Override
  public void execute() {
    m_intake.runIntake(m_speed);
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.runIntake(0);
  }
}