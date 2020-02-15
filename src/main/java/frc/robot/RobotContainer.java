/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.Properties;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.*;
import frc.robot.controlboard.ControlBoard;
import frc.robot.sensors.ColorSensor.ColorDetect;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Queue;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spinner;
import frc.robot.utils.DriveType;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final ControlBoard m_controlBoard;
  private Drivetrain m_drivetrain;
  private Intake m_intake;
  private Shooter m_shooter;
  private Spinner m_spinner;
  private Queue m_queue;
  private final Properties properties;
  private static final String PROPERTIES_NAME = "/robotContainer.properties";

  private final boolean drivetrainEnabled;
  private final boolean intakeEnabled;
  private final boolean shooterEnabled;
  private final boolean spinnerEnabled;
  private final boolean queueEnabled;

  private Command m_defaultDriveCommand;

  private final Command m_autonomousCommand;

  private double m_intakeExtenderSlowify = 0.2;

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_controlBoard = new ControlBoard();
    properties = new Properties();
    try {
      InputStream deployStream = new FileInputStream(Filesystem.getDeployDirectory() + PROPERTIES_NAME);
      properties.load(deployStream);
    } catch(IOException e) {
      e.printStackTrace();
    }

    drivetrainEnabled = Boolean.parseBoolean(properties.getProperty("drivetrain.isEnabled"));
    intakeEnabled = Boolean.parseBoolean(properties.getProperty("intake.isEnabled"));
    shooterEnabled = Boolean.parseBoolean(properties.getProperty("shooter.isEnabled"));
    spinnerEnabled = Boolean.parseBoolean(properties.getProperty("spinner.isEnabled"));
    queueEnabled = Boolean.parseBoolean(properties.getProperty("queue.isEnabled"));

    if (drivetrainEnabled) {m_drivetrain = new Drivetrain();}
    if (intakeEnabled) {m_intake = new Intake();}
    if (shooterEnabled) {m_shooter = new Shooter();}
    if (spinnerEnabled) {m_spinner = new Spinner();}
    if (queueEnabled) {m_queue = new Queue();}

    if (drivetrainEnabled) {
      m_defaultDriveCommand = Commands.simpleArcadeDrive(m_drivetrain, () -> getMove(), () -> getTurn());
      m_drivetrain.setDefaultCommand(m_defaultDriveCommand);
    }
    
    if (intakeEnabled) {
      // Temporary controls for testing intake extender
      // m_intake.setDefaultCommand(Commands.runIntakeExtender_Temp(m_intake, () -> m_controlBoard.extreme.getStickY() * m_intakeExtenderSlowify));
    }

    if (shooterEnabled) {
      // Temporary controls for testing shooter
      // m_shooter.setDefaultCommand(Commands.runShooter(m_shooter, m_controlBoard.extreme::getStickY));
    }
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
    
    if (drivetrainEnabled) {
      // Toggle low gear
      m_controlBoard.xbox.rightBumper.whenPressed(() -> m_drivetrain.setLowGear(true), m_drivetrain).whenReleased(() -> m_drivetrain.setLowGear(false), m_drivetrain);
    }

    if (intakeEnabled) {
      // Start/stop intaking
      m_controlBoard.xbox.yButton.toggleWhenPressed(Commands.intake(m_intake));
    }
    if (shooterEnabled) {
      // Run shooter at a set motor output
      m_controlBoard.extreme.sideButton.whileHeld(Commands.runShooter(m_shooter, () -> 0.5));
    }
    if (spinnerEnabled) {
      // Extend/retract spinner
      m_controlBoard.extreme.baseFrontLeft.whenPressed(Commands.setSpinnerExtended(m_spinner, true));
      m_controlBoard.extreme.baseFrontRight.whenPressed(Commands.setSpinnerExtended(m_spinner, false));

      m_controlBoard.extreme.baseMiddleLeft.whenPressed( new RotationControl (m_spinner, 3));
      m_controlBoard.extreme.baseMiddleRight.whenPressed( new PrecisionControl(m_spinner, ColorDetect.Red));
    }

    if (queueEnabled) {
      //runs the queue. dont really have a button planned for it
      m_controlBoard.extreme.baseFrontRight.whileHeld(Commands.runQueue(m_queue, 0.5));

      //toggles the hard stop on the queue if there is one. also dont have a button for it
      m_controlBoard.extreme.baseFrontLeft.whenHeld(Commands.toggleQueueGate(m_queue));
    }
  }

  private double getMove() {
    double move = -m_controlBoard.xbox.getLeftStickY();
    move = Math.abs(Math.pow(move, 2)) * Math.signum(move);
    return move / 2;
  }

  private double getTurn() {
    double turn = -m_controlBoard.xbox.getRightStickX();
    turn = Math.abs(Math.pow(turn, 2)) * Math.signum(turn);
    return turn / 2;
  }

  public void setDriveType(DriveType driveType) {
    if (drivetrainEnabled) {
      Command driveCommand;
      switch (driveType) {
        case SIMPLE_ARCADE_DRIVE:
          driveCommand = Commands.simpleArcadeDrive(m_drivetrain, () -> getMove(), () -> getTurn());
          break;
        case VELOCITY_ARCADE_DRIVE:
          driveCommand = Commands.velocityArcadeDrive(m_drivetrain, () -> getMove(), () -> getTurn());
          break;
        case ACCELERATION_LIMITED_SIMPLE_ARCADE_DRIVE:
          driveCommand = Commands.accelerationLimitedSimpleArcadeDrive(m_drivetrain, () -> getMove(), () -> getTurn(), 2, 3);
          break;
        case ACCELERATION_LIMITED_VELOCITY_ARCADE_DRIVE:
          driveCommand = Commands.accelerationLimitedVelocityArcadeDrive(m_drivetrain, () -> getMove(), () -> getTurn(), 2, 3);
          break;
        default:
          driveCommand = m_defaultDriveCommand;
          break;
      }
      m_drivetrain.setDefaultCommand(driveCommand);
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autonomousCommand;
  }

  /**
   * saves all the properties in properties files in subsystems. should be called in disabled init and any other time nessescary.
   * <p> <B> <I> subsystesm must run their property saving methods here for them to save unless theyre called elswhere
   */
  public void saveAllProperties() {
    m_drivetrain.saveProperties();
  }
}
