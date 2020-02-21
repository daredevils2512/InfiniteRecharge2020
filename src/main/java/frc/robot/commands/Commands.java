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

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Queue;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spinner;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.interfaces.IClimber;
import frc.robot.subsystems.interfaces.IDrivetrain;
import frc.robot.subsystems.interfaces.IIntake;
import frc.robot.subsystems.interfaces.IMagazine;
import frc.robot.subsystems.interfaces.IQueue;
import frc.robot.subsystems.interfaces.IShooter;
import frc.robot.subsystems.interfaces.ISpinner;
import frc.robot.subsystems.interfaces.ITurret;
import frc.robot.vision.Limelight;
import frc.robot.RobotContainer;
import frc.robot.sensors.ColorSensor.ColorDetect;

/**
 * Definitions for all commands
 * 
 * <p>The reasoning here is that whether a command
 * should be inlined or not should be up to the discretion
 * of whoever is writing command based on their
 * complexity and specific implementation, but
 * anyone using commands should be able to access
 * them all in the same manner, regardless of whether
 * they are inlined or not. This class is meant to
 * make both requirements possible and avoid overflowing
 * the {@link RobotContainer} class with hard-to-read
 * command definitions.
 */
public final class Commands {
  private Commands() {
  }

  /**
   * Simple percent output arcade drive
   * @param drivetrain Drivetrain to use
   * @param moveSupplier Forward speed supplier (-1 to +1)
   * @param turnSupplier Turn speed supplier (-1 to +1)
   * @return New {@link Command}
   */
  public static Command simpleArcadeDrive(IDrivetrain drivetrain, DoubleSupplier moveSupplier, DoubleSupplier turnSupplier) {
    return new RunCommand(() -> drivetrain.simpleArcadeDrive(moveSupplier.getAsDouble(), turnSupplier.getAsDouble()), drivetrain);
  }

  /**
   * Arcade drive with PID velocity control
   * @param drivetrain Drivetrain to use
   * @param moveSupplier Forward speed supplier (-1 to +1)
   * @param turnSupplier Turn speed supplier (-1 to +1)
   * @return New {@link Command}
   */
  public static Command velocityArcadeDrive(IDrivetrain drivetrain, DoubleSupplier moveSupplier, DoubleSupplier turnSupplier) {
    DoubleSupplier velocitySupplier = () -> moveSupplier.getAsDouble() * drivetrain.getMaxSpeed();
    DoubleSupplier angularVelocitySupplier = () -> turnSupplier.getAsDouble() * drivetrain.getMaxAngularSpeed();
    return new RunCommand(() -> drivetrain.velocityArcadeDrive(velocitySupplier.getAsDouble(), angularVelocitySupplier.getAsDouble()), drivetrain);
  }

  public static Command accelerationLimitedSimpleArcadeDrive(IDrivetrain drivetrain, DoubleSupplier moveSupplier, DoubleSupplier turnSupplier, double maxMoveAcceleration, double maxTurnAcceleration) {
    return new AccelerationLimitedSimpleArcadeDrive(drivetrain, moveSupplier, turnSupplier, maxMoveAcceleration, maxTurnAcceleration);
  }

  public static Command accelerationLimitedVelocityArcadeDrive(IDrivetrain drivetrain, DoubleSupplier moveSupplier, DoubleSupplier turnSupplier, double maxMoveAcceleration, double maxTurnAcceleration) {
    return new AccelerationLimitedVelocityArcadeDrive(drivetrain, moveSupplier, turnSupplier, maxMoveAcceleration, maxTurnAcceleration);
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

  //probly temporary
  public static Command climberUp(IClimber climber, Double leftSpeed, Double rightSpeed) {
    return new RunCommand(() -> climber.climb(leftSpeed, rightSpeed), climber);
  }

  public static Command runQueue(IQueue queue, Double speed) {
    return new RunCommand(() -> queue.run(speed, false), queue);
  }

  public static Command toggleQueueGate(IQueue queue) {
    return new InstantCommand(() -> queue.setClosed(!queue.getClosed()), queue);
  }

  /**
   * Extends and starts running the power cell intake
   * @return New {@link Command}
   */
  public static Command startIntaking(IIntake intake, Magazine magazine) {
    return
      new InstantCommand(() -> intake.setExtended(true), intake).andThen(
      new RunCommand(() -> magazine.setSpeed(1), magazine));
  }

  /**
   * Stops running and retracts the power cell intake
   * @return New {@link Command}
   */
  public static Command stopIntaking(IIntake intake, Magazine magazine) {
    return
      new InstantCommand(() -> magazine.setSpeed(0), magazine).andThen(
      new InstantCommand(() -> intake.setExtended(false), intake));
  }

  public static Command runIntake(IIntake intake, IMagazine magazine, double speed) {
    return new RunIntake(intake, magazine, speed);
  }

  public static Command runIntakeExtender_Temp(IIntake intake, DoubleSupplier speedSupplier) {
    return new RunCommand(() -> intake.runExtender(speedSupplier.getAsDouble()), intake);
  }

  public static Command runMagazine(IMagazine magazine, DoubleSupplier speedSupplier) {
    return new RunCommand(() -> magazine.setSpeed(speedSupplier.getAsDouble()), magazine);
  }

  public static Command refillQueue(IMagazine magazine, double magazineSpeed, BooleanSupplier powerCellQueued) {
    return new RefillQueue(magazine, magazineSpeed, powerCellQueued);
  }

  public static Command autoRefillQueue(IMagazine magazine, double magazineSpeed, BooleanSupplier powerCellQueued) {
    return new AutoRefillQueue(magazine, magazineSpeed, powerCellQueued);
  }

  public static Command runQueue(IQueue queue, DoubleSupplier speedSupplier) {
    return new RunCommand(() -> queue.run(speedSupplier.getAsDouble()), queue);
  }

  public static Command feedShooter(IQueue queue, DoubleSupplier queueSpeedSupplier) {
    return new FeedShooter(queue, queueSpeedSupplier);
  }

  public static Command autoFeedShooter(IQueue queue, double queueSpeed, IntSupplier magazinePowerCellCountSupplier) {
    return new AutoFeedShooter(queue, queueSpeed, magazinePowerCellCountSupplier);
  }

  public static Command moveTurret(ITurret turret, DoubleSupplier speedSupplier) {
    return new RunCommand(() -> turret.setSpeed(speedSupplier.getAsDouble()), turret);
  }

  public static Command findTarget(ITurret turret, Limelight limelight, double angleTolerance) {
    return new FindTarget(turret, limelight, angleTolerance);
  }

  /**
   * Set shooter percent output
   * @param shooter
   * @return New {@link Command}
   */
  public static Command runShooter(IShooter shooter, DoubleSupplier speedSupplier) {
    System.out.println(speedSupplier.getAsDouble());
    return new RunCommand(() -> shooter.setPercentOutput(speedSupplier.getAsDouble()), shooter);
    // return new RunCommand(() -> shooter.setPercentOutput(speedSupplier.getAsDouble()), shooter);
  }

  public static Command setShooterVelocity(IShooter shooter, DoubleSupplier velocitySupplier) {
    return new RunCommand(() -> shooter.setTargetVelocity(velocitySupplier.getAsDouble()), shooter);
  }

  public static Command setShooterAngle(IShooter shooter, DoubleSupplier angleSupplier) {
    return new RunCommand(() -> shooter.setTargetAngle(angleSupplier.getAsDouble()), shooter);
  }

  public static Command stopShooter(IShooter shooter) {
    return new InstantCommand(() -> shooter.stop(), shooter);
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

  public static Command followPath(IDrivetrain drivetrain, String file) {
    Trajectory trajectory;
    try {
      Path path = Filesystem.getDeployDirectory().toPath().resolve("paths/" + file);
      trajectory = TrajectoryUtil.fromPathweaverJson(path);
    } catch(IOException e) {
      trajectory = null;
      e.printStackTrace();
    }
    return new RamseteCommand(trajectory, drivetrain::getPose , new RamseteController(),
      drivetrain.getKinematics(), drivetrain::voltageTank , drivetrain)
      .andThen(() -> drivetrain.simpleArcadeDrive(0, 0));
  }
}
