package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;
import frc.robot.vision.LimelightPipeline;

public abstract class TrackTargetCommand extends CommandBase {
  protected final Turret m_turret;
  protected final LimelightPipeline m_pipeline;

  protected TrackTargetCommand(Turret turret, LimelightPipeline pipeline) {
    m_turret = turret;
    m_pipeline = pipeline;
  }

  @Override
  public void end(boolean interrupted) {
    m_turret.setSpeed(0);
  }
}
