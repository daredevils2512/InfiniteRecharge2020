/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.interfaces.IIntake;
import frc.robot.utils.CommandLogger;

public class ToggleIntakeExtended extends CommandLogger {

  private IIntake m_intake;

  private boolean wantsExtended;
  /**
   * Creates a new ToggleIntakeExtended.
   */
  public ToggleIntakeExtended(IIntake intake) {
    m_intake = intake;

    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setMotionMagicEnabled(true);
    wantsExtended = !m_intake.getExtended();
    m_intake.setExtended(wantsExtended);
    m_logger.fine("setting intake extended to" + wantsExtended);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.setExtended(wantsExtended);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setMotionMagicEnabled(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_intake.isMotionMagicFinished();
  }
}
