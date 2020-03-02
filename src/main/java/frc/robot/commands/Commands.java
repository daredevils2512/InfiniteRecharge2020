/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;
import java.util.logging.Level;
import java.util.logging.Logger;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.spline.Spline;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.ControlVectorList;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.CompressorManager;
import frc.robot.subsystems.interfaces.IClimber;
import frc.robot.subsystems.interfaces.ICompressorManager;
import frc.robot.subsystems.interfaces.IDrivetrain;
import frc.robot.subsystems.interfaces.IIntake;
import frc.robot.subsystems.interfaces.IMagazine;
import frc.robot.subsystems.interfaces.IQueue;
import frc.robot.subsystems.interfaces.IShooter;
import frc.robot.subsystems.interfaces.ISpinner;
import frc.robot.subsystems.interfaces.ITurret;
import frc.robot.utils.MagazinePowerCellCounter;
import frc.robot.vision.PiTable;
import frc.robot.vision.Limelight;
import frc.robot.vision.LimelightLEDMode;
import frc.robot.RobotContainer;
import frc.robot.sensors.ColorSensor.ColorDetect;

/**
 * Definitions for all commands
 * 
 * <p>
 * The reasoning here is that whether a command should be inlined or not should
 * be up to the discretion of whoever is writing command based on their
 * complexity and specific implementation, but anyone using commands should be
 * able to access them all in the same manner, regardless of whether they are
 * inlined or not. This class is meant to make both requirements possible and
 * avoid overflowing the {@link RobotContainer} class with hard-to-read command
 * definitions.
 */
public final class Commands {
  private Commands() {
  }

  public static Command setLimelightLEDMode(Limelight limelight, LimelightLEDMode ledMode) {
    return new InstantCommand(() -> limelight.setLEDMode(ledMode));
  }

  /**
   * Sets the limelight LED mode to force OFF if not currently set to OFF, otherwise revert to mode set by pipeline
   * @param limelight Limelight to set LED mode on
   * @return New {@link Command}
   */
  public static Command toggleLimelightLEDForceOff(Limelight limelight) {
    return new InstantCommand(() -> {
      limelight.setLEDMode(limelight.getLEDMode() == LimelightLEDMode.OFF ? LimelightLEDMode.PIPELINE : LimelightLEDMode.OFF);
    });
  }

  /**
   * Simple percent output arcade drive
   * 
   * @param drivetrain   Drivetrain to use
   * @param moveSupplier Forward speed supplier (-1 to +1)
   * @param turnSupplier Turn speed supplier (-1 to +1)
   * @return New {@link Command}
   */
  public static Command simpleArcadeDrive(IDrivetrain drivetrain, DoubleSupplier moveSupplier,
      DoubleSupplier turnSupplier) {
    return new RunCommand(() -> drivetrain.simpleArcadeDrive(moveSupplier.getAsDouble(), turnSupplier.getAsDouble()),
        drivetrain);
  }

  /**
   * Arcade drive with PID velocity control
   * 
   * @param drivetrain   Drivetrain to use
   * @param moveSupplier Forward speed supplier (-1 to +1)
   * @param turnSupplier Turn speed supplier (-1 to +1)
   * @return New {@link Command}
   */
  public static Command velocityArcadeDrive(IDrivetrain drivetrain, DoubleSupplier moveSupplier,
      DoubleSupplier turnSupplier) {
    DoubleSupplier velocitySupplier = () -> moveSupplier.getAsDouble() * drivetrain.getMaxSpeed();
    DoubleSupplier angularVelocitySupplier = () -> turnSupplier.getAsDouble() * drivetrain.getMaxAngularSpeed();
    return new RunCommand(
        () -> drivetrain.velocityArcadeDrive(velocitySupplier.getAsDouble(), angularVelocitySupplier.getAsDouble()),
        drivetrain);
  }

  public static Command accelerationLimitedSimpleArcadeDrive(IDrivetrain drivetrain, DoubleSupplier moveSupplier,
      DoubleSupplier turnSupplier, double maxMoveAcceleration, double maxTurnAcceleration) {
    return new AccelerationLimitedSimpleArcadeDrive(drivetrain, moveSupplier, turnSupplier, maxMoveAcceleration,
        maxTurnAcceleration);
  }

  public static Command accelerationLimitedVelocityArcadeDrive(IDrivetrain drivetrain, DoubleSupplier moveSupplier,
      DoubleSupplier turnSupplier, double maxMoveAcceleration, double maxTurnAcceleration) {
    return new AccelerationLimitedVelocityArcadeDrive(drivetrain, moveSupplier, turnSupplier, maxMoveAcceleration,
        maxTurnAcceleration);
  }

  public static Command driveStraight(IDrivetrain drivetrain, double distance) {
    return new DriveStraight(drivetrain, distance);
  }

  public static Command setDrivingInverted(IDrivetrain drivetrain, boolean wantsInverted) {
    return new InstantCommand(() -> drivetrain.setDrivingInverted(wantsInverted), drivetrain);
  }

  public static Command toggleInvertedDriving(IDrivetrain drivetrain) {
    return new InstantCommand(() -> drivetrain.setDrivingInverted(!drivetrain.getDrivingInverted()), drivetrain);
  }

  public static Command drivetrainSetLowGear(IDrivetrain drivetrain, boolean wantsLowGear) {
    return new InstantCommand(() -> drivetrain.setLowGear(wantsLowGear), drivetrain);
  }

  public static Command drivetrainToggleLowGear(IDrivetrain drivetrain) {
    return new InstantCommand(() -> drivetrain.setLowGear(!drivetrain.getLowGear()), drivetrain);
  }

  // probly temporary
  public static Command climberUp(IClimber climber, DoubleSupplier leftSpeed, DoubleSupplier rightSpeed) {
    return new RunCommand(() -> climber.climb(leftSpeed.getAsDouble(), rightSpeed.getAsDouble()), climber);
  }

  /**
   * Extends and starts running the power cell intake
   * 
   * @return New {@link Command}
   */
  public static Command startIntaking(IIntake intake, IMagazine magazine) {
    return new InstantCommand(() -> intake.setExtended(true), intake)
        .andThen(new RunCommand(() -> magazine.setSpeed(1), magazine));
  }

  /**
   * Stops running and retracts the power cell intake
   * 
   * @return New {@link Command}
   */
  public static Command stopIntaking(IIntake intake, IMagazine magazine) {
    return new InstantCommand(() -> magazine.setSpeed(0), magazine)
        .andThen(new InstantCommand(() -> intake.setExtended(false), intake));
  }

  public static Command runIntake(IIntake intake, double speed) {
    return new RunIntake(intake, speed);
  }

  public static Command runIntakeExtender_Temp(IIntake intake, DoubleSupplier speedSupplier) {
    return new RunCommand(() -> intake.runExtender(speedSupplier.getAsDouble()), intake);
  }

  public static Command setIntakeExtended(IIntake intake, boolean wantsExtended) {
    return new InstantCommand(() -> intake.setExtended(wantsExtended));
  }

  public static Command toggleIntakeExtended(IIntake intake) {
    return new ToggleIntakeExtended(intake);
  }

  public static Command runMagazine(IMagazine magazine, double speed) {
    return new RunMagazine(magazine, speed);
  }

  public static Command refillQueue(IMagazine magazine, double magazineSpeed,
      IntSupplier magazinePowerCellCountSupplier, BooleanSupplier queueHasPowerCellSupplier) {
    return new RefillQueue(magazine, magazineSpeed, magazinePowerCellCountSupplier, queueHasPowerCellSupplier);
  }

  public static Command intakeCommand(IIntake intake, Supplier<Double> intakeAxis, double intakeSpeed, IMagazine magazine,
  double magazineSpeed, Supplier<Double> extenderSpeed, double extenderMaxSpeed, Supplier<Boolean> extended,
  Supplier<Boolean> shouldExtend) {
    return new IntakeCommand(intake, intakeAxis, intakeSpeed, magazine, magazineSpeed, extenderSpeed, extenderMaxSpeed, extended, shouldExtend);
  }

  public static Command runQueue(IQueue queue, double speed) {
    return new ManualRunQueue(queue, speed);
  }

  public static Command feedShooter(IQueue queue, DoubleSupplier queueSpeedSupplier) {
    return new FeedShooter(queue, queueSpeedSupplier);
  }

  public static Command autoFeedShooter(IQueue queue, double queueSpeed, IShooter shooter, double tolerance) {
    return new AutoFeedShooter(queue, queueSpeed, shooter, tolerance);
  }

  public static Command moveTurret(ITurret turret, DoubleSupplier speedSupplier) {
    return new RunCommand(() -> turret.setSpeed(speedSupplier.getAsDouble()), turret);
  }

  public static Command runTurretPosition(ITurret turret, double position) {
    System.out.println("ran turret position" + position);
    return new RunCommand(() -> turret.runPosition(position), turret);
  }

  public static Command findTarget(ITurret turret) {
    Logger.getGlobal().log(Level.INFO, "finding traget");
    return new FindTarget(turret);
  }

  /**
   * Set shooter percent output
   * 
   * @param shooter
   * @return New {@link Command}
   */
  public static Command runShooter(IShooter shooter, DoubleSupplier speedSupplier) {
    return new RunCommand(() -> shooter.setPercentOutput(speedSupplier.getAsDouble()), shooter);
    // return new RunCommand(() ->
    // shooter.setPercentOutput(speedSupplier.getAsDouble()), shooter);
  }

  public static Command setShooterVelocity(IShooter shooter, Supplier<Double> speed) {
    return new RunShooterPID(shooter, speed);
  }

  public static Command setShooterAngle(IShooter shooter, DoubleSupplier angleSupplier) {
    return new RunCommand(() -> shooter.setTargetAngle(angleSupplier.getAsDouble()), shooter);
  }

  public static Command runHood(IShooter shooter, DoubleSupplier supplier) {
    return new RunCommand(() -> shooter.setHoodSpeed(supplier.getAsDouble()))
        .andThen(new RunCommand(() -> shooter.setHoodSpeed(0.0)));
  }

  public static Command stopShooter(IShooter shooter) {
    return new InstantCommand(shooter::stop, shooter);
  }

  public static Command setSpinnerExtended(ISpinner spinner, boolean wantsExtended) {
    return new InstantCommand(() -> spinner.setExtended(wantsExtended), spinner);
  }

  public static Command rotationControl(ISpinner spinner, double rotations) {
    return new RotationControl(spinner, rotations);
  }

  public static Command precisionControl(ISpinner spinner, ColorDetect targetColor) {
    return new PrecisionControl(spinner, targetColor);
  }

  public static Command toggleCompressor(ICompressorManager compressor) {
    return new InstantCommand(() -> compressor.toggleCompressor());
  }

  public static Command findBall(IDrivetrain drivetrain, PiTable table) {
    Translation2d translation = table.getClosestBallPose().getTranslation();
    TrajectoryConfig config = new TrajectoryConfig(3.0, 3.0);
    ControlVectorList vectors = new ControlVectorList();
    Spline.ControlVector vector = new Spline.ControlVector(new double[]{translation.getX()}, new double[]{translation.getY()});
    vectors.add(vector);
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(vectors, config);

    return new RamseteCommand(trajectory, drivetrain::getPose, new RamseteController(), drivetrain.getFeedForward(),
        drivetrain.getKinematics(), drivetrain::getWheelSpeeds, drivetrain.getLeftController(),
        drivetrain.getRightController(), drivetrain::voltageTank, drivetrain)
        .andThen(() -> drivetrain.simpleArcadeDrive(0, 0));
    
  }

  public static Command followPath(IDrivetrain drivetrain, String file) {
    Trajectory trajectory;
    try {
      Path path = Filesystem.getDeployDirectory().toPath().resolve("paths/" + file);
      trajectory = TrajectoryUtil.fromPathweaverJson(path);
    } catch(IOException e) {
      trajectory = null;
      e.printStackTrace();
    }
    return new RamseteCommand(trajectory, drivetrain::getPose, new RamseteController(), drivetrain.getFeedForward(),
        drivetrain.getKinematics(), drivetrain::getWheelSpeeds, drivetrain.getLeftController(),
        drivetrain.getRightController(), drivetrain::voltageTank, drivetrain)
        .andThen(() -> drivetrain.simpleArcadeDrive(0, 0));
  }
}
