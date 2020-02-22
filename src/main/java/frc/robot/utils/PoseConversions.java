package frc.robot.utils;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.Constants;

public class PoseConversions {
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

  /**
   * Convert limelight relative pose to turret relative pose
   * 
   * <p>The Y axis of the turret relative pose is parallel to the flywheel's axis
   * @param limelightRelativePose Limelight relative pose
   * @return Turret relative pose
   */
  public static Pose2d turretLimelightToTurret(Pose2d limelightRelativePose) {
    return limelightRelativePose.relativeTo(Constants.TURRET_LIMELIGHT_TURRET_RELATIVE);
  }

  /**
   * Convert turret relative pose to robot relative pose
   * @param turretRelativePose Turret relative pose
   * @return Robot relative pose
   */
  public static Pose2d turretToRobot(Pose2d turretRelativePose, Rotation2d turretRotation) {
    Pose2d newPose = turretRelativePose.relativeTo(Constants.TURRET_ROBOT_RELATIVE);
    Transform2d turretRotationTransform = new Transform2d(new Translation2d(), turretRotation);
    newPose.plus(turretRotationTransform);
    return newPose;
  }
}
