/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;
import frc.robot.vision.Limelight;
import frc.robot.vision.Limelight.Pipeline;

public class FindTarget extends CommandBase {
  private Turret m_turret;
  private Limelight m_limelight;

  /**
   * Creates a new FindTarget.
   */
  public FindTarget(Turret turret, Limelight limelight) {
    m_turret = turret;
    m_limelight = limelight;
    addRequirements(m_turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_limelight.setPipeline(Pipeline.PowerCellTopTarget);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_limelight.hasTarget()) {
      m_turret.setTargetAngle(m_turret.getAngle() + m_limelight.tx());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
