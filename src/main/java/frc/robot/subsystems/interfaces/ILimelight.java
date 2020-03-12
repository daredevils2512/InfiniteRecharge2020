package frc.robot.subsystems.interfaces;

import frc.robot.vision.LimelightLEDMode;
import frc.robot.vision.Limelight.Pipeline;

public interface ILimelight {
  public Pipeline getDefaultPipeline();
  public void setPipeline(Pipeline pipeline);
  public void setLEDMode(LimelightLEDMode ledMode);
  public LimelightLEDMode getLEDMode();
  public boolean hasTarget();
  public double tx();
  public double ty();
  public double ta();
  public double ts();
  public double tl();
  public int tshort();
  public int tlong();
  public int thor();
  public int tvert();
  public double getLastPosition();
  public double getDistanceToTarget();
}
