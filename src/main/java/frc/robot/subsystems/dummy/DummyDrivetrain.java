package frc.robot.subsystems.dummy;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import frc.robot.subsystems.interfaces.IDrivetrain;

public class DummyDrivetrain implements IDrivetrain {

  @Override
  public double getMaxSpeed() {
    return 0;
  }

  @Override
  public double getMaxAngularSpeed() {
    return 0;
  }

  @Override
  public double getMaxAcceleration() {
    return 0;
  }

  @Override
  public boolean getDrivingInverted() {
    return false;
  }

  @Override
  public void setDrivingInverted(boolean wantsInverted) {

  }

  @Override
  public double getLeftDistance() {
    return 0;
  }

  @Override
  public double getRightDistance() {
    return 0;
  }

  @Override
  public double getLeftVelocity() {
    return 0;
  }

  @Override
  public double getRightVelocity() {
    return 0;
  }

  @Override
  public DifferentialDriveKinematics getKinematics() {
    return null;
  }

  @Override
  public double getHeading() {
    return 0;
  }

  @Override
  public void setLowGear(boolean wantsLowGear) {
  }

  @Override
  public boolean getLowGear() {
    return false;
  }

  @Override
  public Pose2d getPose() {
    return null;
  }

  @Override
  public void resetPose() {
  }

  @Override
  public void resetPose(Translation2d translation) {
  }

  @Override
  public void resetPose(Rotation2d rotation) {
  }

  @Override
  public void resetPose(Pose2d pose) {
  }

  @Override
  public void simpleArcadeDrive(double move, double turn) {
  }

  @Override
  public void voltageTank(double left, double right) {
  }

  @Override
  public void velocityArcadeDrive(double velocity, double angularVelocity) {
  }

  @Override
  public void setWheelSpeeds(double left, double right) {
  }

  @Override
  public Map<String, Object> getValues() {
    return null;
  }

  @Override
  public void saveProperties() {
  }

  @Override
  public SimpleMotorFeedforward getFeedForward() {
    return new SimpleMotorFeedforward(0, 0);
  }

  @Override
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds();
  }

  @Override
  public PIDController getLeftController() {
    return new PIDController(0.0, 0.0, 0.0);
  }

  @Override
  public PIDController getRightController() {
    return new PIDController(0.0, 0.0, 0.0);
  }
}