/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Queue;
import frc.robot.subsystems.Shooter;
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
  private static final class IntakeCommand extends CommandBase {
    private final Intake m_intake;

    public IntakeCommand(Intake intake) {
      m_intake = intake;
      addRequirements(m_intake);
    }

    @Override
    public void initialize() {
      m_intake.setExtended(true);
    }

    @Override
    public void execute() {
      m_intake.run(1);
    }

    @Override
    public void end(boolean interrupted) {
      m_intake.run(0);
      m_intake.setExtended(false);
    }
  }

  private Commands() {

  }

  public static Command arcadeDrive(Drivetrain drivetrain, DoubleSupplier moveSupplier, DoubleSupplier turnSupplier) {
    return new RunCommand(() -> drivetrain.arcadeDrive(moveSupplier.getAsDouble(), turnSupplier.getAsDouble()), drivetrain);
  }

  //probly temporary
  public static Command climberUp(Climber climber, Double leftSpeed, Double rightSpeed) {
    return new RunCommand(() -> climber.climb(leftSpeed, rightSpeed), climber);
  }

  public static Command runQueue(Queue queue, Double speed) {
    return new RunCommand(() -> queue.run(speed), queue);
  }

  public static Command toggleQueueGate(Queue queue) {
    boolean isQueueClosed = queue.getIsClosed();
    return new InstantCommand(() -> queue.setClosed(!isQueueClosed), queue);
  }

  /**
   * Extends and starts running the power cell intake
   * @return New {@link Command}
   */
  public static Command startIntaking(Intake intake) {
    return
      new InstantCommand(() -> intake.setExtended(true), intake).andThen(
      new RunCommand(() -> intake.run(1), intake));
  }

  /**
   * Stops running and retracts the power cell intake
   * @return New {@link Command}
   */
  public static Command stopIntaking(Intake intake) {
    return
      new InstantCommand(() -> intake.run(0), intake).andThen(
      new InstantCommand(() -> intake.setExtended(false), intake));
  }

  public static Command runIntakeExtender_Temp(Intake intake, DoubleSupplier speedSupplier) {
    return new RunCommand(() -> intake.runExtender_Temp(speedSupplier.getAsDouble()), intake);
  }

  /**
   * Extends and starts running the power cell intake
   * 
   * <p>Stops running and retracts the intake when interrupted
   * @return New {@link Command}
   */
  public static Command intake(Intake intake) {
    return new IntakeCommand(intake);
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
    return new StartEndCommand(() -> shooter.percentOutput(speedSupplier.getAsDouble()), () -> shooter.percentOutput(0), shooter);
  }

  public static Command setShooterVelocity(Shooter shooter, DoubleSupplier velocitySupplier) {
    return null; // TODO: Implement shooter set velocity command
  }
}
