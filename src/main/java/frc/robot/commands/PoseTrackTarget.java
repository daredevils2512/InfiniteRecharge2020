package frc.robot.commands;

import frc.robot.subsystems.Turret;
import frc.robot.utils.PoseConversions;
import frc.robot.vision.LimelightPipeline;
import frc.robot.vision.LimelightUtil;

public class PoseTrackTarget extends TrackTargetCommand {
  public PoseTrackTarget(Turret turret, LimelightPipeline pipeline) {
    super(turret, pipeline);
  }

  @Override
  public void execute() {
    double horizontalOffset = LimelightUtil.getHorizontalOffset(m_pipeline);
  }
}
