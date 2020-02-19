package frc.robot.utils;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

public class PoseConversions {
  // TODO: Determine turret limelight to turret transform
  private static final Transform2d TURRET_LIMELIGHT_TO_TURRET = new Transform2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0));
  // TODO: Determine turret to robot transform
  private static final Transform2d TURRET_TO_ROBOT = new Transform2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0));

  private PoseConversions() {

  }

  /**
   * Get the target's pose relative to the limelight based on the calculated distance and horizontal offset
   * @param distance Distance to target in meters
   * @param horizontalOffset Angle to the target in degrees
   * @return Target's pose relative to the limelight
   */
  public static Pose2d getLimelightTargetPose(double distance, double horizontalOffset) {
    Rotation2d rotationToTarget = Rotation2d.fromDegrees(horizontalOffset);
    Translation2d translationToTarget = new Translation2d(distance, 0).rotateBy(rotationToTarget);
    return new Pose2d(translationToTarget, rotationToTarget);
  }

  public static Pose2d turretLimelightToTurret(Pose2d limelightRelativePose) {
    return limelightRelativePose.transformBy(TURRET_LIMELIGHT_TO_TURRET);
  }

  public static Pose2d turretToRobot(Pose2d turretRelativePose) {
    return turretRelativePose.transformBy(TURRET_TO_ROBOT);
  }
}
