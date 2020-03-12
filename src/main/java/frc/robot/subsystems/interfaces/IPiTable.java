package frc.robot.subsystems.interfaces;

import edu.wpi.first.wpilibj.geometry.Pose2d;

public interface IPiTable {
  public boolean hasTarget();
  public int getNumberOfTargets();
  public double getXOffset(double[] entry);
  public double getYOffset(double[] entry);
  public double getDistance(double[] entry);
  public double getX(double[] entry);
  public double getY(double[] entry);
  public double getSize(double[] entry);
  public Pose2d getPose(double[] entry);
  public double[] getClosestTarget();
  public double[] getTarget(int ball);
  public double[] getLastClosestTarget();
  public Pose2d getClosestBallPose();
}