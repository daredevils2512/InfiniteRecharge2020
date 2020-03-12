package frc.robot.subsystems.dummy;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import frc.robot.subsystems.interfaces.IPiTable;

public class DummyPiTable implements IPiTable {

  @Override
  public boolean hasTarget() {
    return false;
  }

  @Override
  public int getNumberOfTargets() {
    return 0;
  }

  @Override
  public double getXOffset(double[] entry) {
    return 0;
  }

  @Override
  public double getYOffset(double[] entry) {
    return 0;
  }

  @Override
  public double getDistance(double[] entry) {
    return 0;
  }

  @Override
  public double getX(double[] entry) {
    return 0;
  }

  @Override
  public double getY(double[] entry) {
    return 0;
  }

  @Override
  public double getSize(double[] entry) {
    return 0;
  }

  @Override
  public Pose2d getPose(double[] entry) {
    return null;
  }

  @Override
  public double[] getClosestTarget() {
    return null;
  }

  @Override
  public double[] getTarget(int ball) {
    return null;
  }

  @Override
  public double[] getLastClosestTarget() {
    return null;
  }

  @Override
  public Pose2d getClosestBallPose() {
    return null;
  }
}