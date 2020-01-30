/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.*;
import frc.robot.controlboard.ControlBoard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Queue;
import frc.robot.subsystems.Shooter;
import frc.robot.vision.Limelight.Pipeline;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final ControlBoard m_controlBoard = new ControlBoard();
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final Intake m_intake = new Intake();
  private final Shooter m_shooter = new Shooter();
  private final Queue m_queue = new Queue();

  private final Command m_autonomousCommand;

  private double m_intakeExtenderSlowify = 0.2;

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_drivetrain.setDefaultCommand(Commands.arcadeDrive(m_drivetrain, m_controlBoard.xbox::getLeftStickY, m_controlBoard.xbox::getRightStickX));
    
    // Temporary controls for testing intake extender
    m_intake.setDefaultCommand(Commands.runIntakeExtender_Temp(m_intake, () -> m_controlBoard.extreme.getStickY() * m_intakeExtenderSlowify));

    configureButtonBindings();

    m_autonomousCommand = null;
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Toggle low gear
    m_controlBoard.xbox.rightBumper.whenPressed(() -> m_drivetrain.setLowGear(true), m_drivetrain).whenReleased(() -> m_drivetrain.setLowGear(false), m_drivetrain);

    // Start/stop intaking
    m_controlBoard.xbox.yButton.toggleWhenPressed(Commands.intake(m_intake));

    m_controlBoard.xbox.aButton.whileHeld(new FollowBall(m_drivetrain, Pipeline.PowerCells));

    // Run shooter at a set motor output
    m_controlBoard.extreme.sideButton.whileHeld(Commands.runShooter(m_shooter, () -> 0.5));

    //runs the queue. dont really have a button planned for it
    m_controlBoard.extreme.baseFrontRight.whileHeld(Commands.runQueue(m_queue, 0.5));

    //toggles the hard stop on the queue if there is one. also dont have a button for it
    m_controlBoard.extreme.baseFrontLeft.whenHeld(Commands.toggleQueueGate(m_queue));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autonomousCommand;
  }
}
