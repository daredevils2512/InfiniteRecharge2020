package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;
import frc.robot.vision.LimelightPipeline;
import frc.robot.vision.LimelightUtil;

public class SimpleTrackTarget extends TrackTargetCommand {
  public SimpleTrackTarget(Turret turret, LimelightPipeline pipeline) {
    super(turret, pipeline);
  }

  @Override
  public void execute() {
    double currentAngle = m_turret.getAngle();
    double horizontalOffset = LimelightUtil.getHorizontalOffset(m_pipeline);
    double targetAngle = currentAngle + horizontalOffset;
    m_turret.setTargetAngle(targetAngle);
  }
}
