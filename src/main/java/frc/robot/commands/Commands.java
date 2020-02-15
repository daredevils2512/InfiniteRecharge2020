/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Queue;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spinner;
import frc.robot.subsystems.Turret;
import frc.robot.RobotContainer;

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
  public static Command simpleArcadeDrive(Drivetrain drivetrain, DoubleSupplier moveSupplier, DoubleSupplier turnSupplier) {
    return new RunCommand(() -> drivetrain.simpleArcadeDrive(moveSupplier.getAsDouble(), turnSupplier.getAsDouble()), drivetrain);
  }

  /**
   * Arcade drive with PID velocity control
   * @param drivetrain Drivetrain to use
   * @param moveSupplier Forward speed supplier (-1 to +1)
   * @param turnSupplier Turn speed supplier (-1 to +1)
   * @return New {@link Command}
   */
  public static Command velocityArcadeDrive(Drivetrain drivetrain, DoubleSupplier moveSupplier, DoubleSupplier turnSupplier) {
    DoubleSupplier velocitySupplier = () -> moveSupplier.getAsDouble() * drivetrain.getMaxSpeed();
    DoubleSupplier angularVelocitySupplier = () -> turnSupplier.getAsDouble() * drivetrain.getMaxAngularSpeed();
    return new RunCommand(() -> drivetrain.velocityArcadeDrive(velocitySupplier.getAsDouble(), angularVelocitySupplier.getAsDouble()), drivetrain);
  }

  public static Command accelerationLimitedSimpleArcadeDrive(Drivetrain drivetrain, DoubleSupplier moveSupplier, DoubleSupplier turnSupplier, double maxMoveAcceleration, double maxTurnAcceleration) {
    return new AccelerationLimitedSimpleArcadeDrive(drivetrain, moveSupplier, turnSupplier, maxMoveAcceleration, maxTurnAcceleration);
  }

  public static Command accelerationLimitedVelocityArcadeDrive(Drivetrain drivetrain, DoubleSupplier moveSupplier, DoubleSupplier turnSupplier, double maxMoveAcceleration, double maxTurnAcceleration) {
    return new AccelerationLimitedVelocityArcadeDrive(drivetrain, moveSupplier, turnSupplier, maxMoveAcceleration, maxTurnAcceleration);
  }

  //probly temporary
  public static Command climberUp(Climber climber, Double leftSpeed, Double rightSpeed) {
    return new RunCommand(() -> climber.climb(leftSpeed, rightSpeed), climber);
  }

  public static Command runQueue(Queue queue, Double speed) {
    return new RunCommand(() -> queue.run(speed, false), queue);
  }

  public static Command toggleQueueGate(Queue queue) {
    boolean isQueueClosed = queue.getClosed();
    return new InstantCommand(() -> queue.setClosed(!isQueueClosed), queue);
  }

  /**
   * Extends and starts running the power cell intake
   * @return New {@link Command}
   */
  public static Command startIntaking(Intake intake, Magazine magazine) {
    return
      new InstantCommand(() -> intake.setExtended(true), intake).andThen(
      new RunCommand(() -> magazine.setSpeed(1), magazine));
  }

  /**
   * Stops running and retracts the power cell intake
   * @return New {@link Command}
   */
  public static Command stopIntaking(Intake intake, Magazine magazine) {
    return
      new InstantCommand(() -> magazine.setSpeed(0), magazine).andThen(
      new InstantCommand(() -> intake.setExtended(false), intake));
  }

  public static Command runIntake(Intake intake, Magazine magazine, double speed) {
    return new RunIntake(intake, magazine, speed);
  }

  public static Command runIntakeExtender_Temp(Intake intake, DoubleSupplier speedSupplier) {
    return new RunCommand(() -> intake.runExtender(speedSupplier.getAsDouble()), intake);
  }

  public static Command refillQueue(Magazine magazine, double magazineSpeed, BooleanSupplier powerCellQueued) {
    return new RefillQueue(magazine, magazineSpeed, powerCellQueued);
  }

  public static Command autoRefillQueue(Magazine magazine, double magazineSpeed, BooleanSupplier powerCellQueued) {
    return new AutoRefillQueue(magazine, magazineSpeed, powerCellQueued);
  }

  public static Command moveTurret(Turret turret, DoubleSupplier speedSupplier) {
    return new RunCommand(() -> turret.setSpeed(speedSupplier.getAsDouble()), turret);
  }

  /**
   * Set shooter percent output
   * @param shooter
   * @return New {@link Command}
   */
  public static Command runShooter(Shooter shooter, DoubleSupplier speedSupplier) {
    return new RunCommand(() -> shooter.setPercentOutput(speedSupplier.getAsDouble()), shooter);
  }

  public static Command setShooterVelocity(Shooter shooter, DoubleSupplier velocitySupplier) {
    return new RunCommand(() -> shooter.setTargetVelocity(velocitySupplier.getAsDouble()), shooter);
  }

  public static Command setShooterAngle(Shooter shooter, DoubleSupplier angleSupplier) {
    return new RunCommand(() -> shooter.setTargetAngle(angleSupplier.getAsDouble()), shooter);
  }

  public static Command stopShooter(Shooter shooter) {
    return new InstantCommand(() -> shooter.stop(), shooter);
  }

  public static Command setSpinnerExtended(Spinner spinner, boolean wantsExtended) {
    return new InstantCommand(() -> spinner.setExtended(wantsExtended), spinner);
  }

}
