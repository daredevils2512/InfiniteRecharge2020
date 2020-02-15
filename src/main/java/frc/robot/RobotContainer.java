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
import java.util.logging.*;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.*;
import frc.robot.controlboard.ControlBoard;
import frc.robot.sensors.ColorSensor.ColorDetect;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Queue;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spinner;
import frc.robot.subsystems.*;
import frc.robot.utils.DriveType;
import frc.robot.vision.HexagonPosition;
import frc.robot.vision.Limelight;
import frc.robot.vision.Limelight.Pipeline;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final ControlBoard m_controlBoard;
  private HexagonPosition m_hexagonPosition;
  private Limelight m_limelight;
  private Drivetrain m_drivetrain;
  private Intake m_intake;
  private Shooter m_shooter;
  private Spinner m_spinner;
  private Queue m_queue;
  private Turret m_turret;
  private Magazine m_magazine;
  private Climber m_climber;
  private CompressorManager m_compressor;
  private final Properties properties;
  private static final String PROPERTIES_NAME = "/robotContainer.properties";

  private static Logger logger = Logger.getLogger(RobotContainer.class.getName());

  private static Logger climberLog = Logger.getLogger(Climber.class.getName());
  private static Logger compressorLog = Logger.getLogger(Compressor.class.getName());
  private static Logger drivetrainLog = Logger.getLogger(Drivetrain.class.getName());
  private static Logger intakeLog = Logger.getLogger(Intake.class.getName());
  private static Logger magazineLog = Logger.getLogger(Magazine.class.getName());
  private static Logger queueLog = Logger.getLogger(Queue.class.getName());
  private static Logger shooterLog = Logger.getLogger(Shooter.class.getName());
  private static Logger spinnerLog = Logger.getLogger(Spinner.class.getName());
  private static Logger turretLog = Logger.getLogger(Turret.class.getName());
  private static Logger limelightLog = Logger.getLogger(Limelight.class.getName());

  private Level climberLogLevel;
  private Level compressorLogLevel;
  private Level drivetrainLogLevel;
  private Level intakeLogLevel;
  private Level magazineLogLevel;
  private Level queueLogLevel;
  private Level shooterLogLevel;
  private Level spinnerLogLevel;
  private Level turretLogLevel;
  private Level limelightLogLevel;

  private final boolean limelightEnabled;
  private final boolean drivetrainEnabled;
  private final boolean intakeEnabled;
  private final boolean shooterEnabled;
  private final boolean spinnerEnabled;
  private final boolean queueEnabled;
  private final boolean turretEnabled;
  private final boolean magazineEnabled;
  private final boolean climberEnabled;
  private final boolean compressorEnabled;

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
      logger.info("succesfuly loaded");
    } catch(IOException e) {
      logger.log(Level.SEVERE, "failed to load", e);
    }

    limelightEnabled = Boolean.parseBoolean(properties.getProperty("limelight.isEnabled"));
    drivetrainEnabled = Boolean.parseBoolean(properties.getProperty("drivetrain.isEnabled"));
    intakeEnabled = Boolean.parseBoolean(properties.getProperty("intake.isEnabled"));
    shooterEnabled = Boolean.parseBoolean(properties.getProperty("shooter.isEnabled"));
    spinnerEnabled = Boolean.parseBoolean(properties.getProperty("spinner.isEnabled"));
    queueEnabled = Boolean.parseBoolean(properties.getProperty("queue.isEnabled"));
    turretEnabled = Boolean.parseBoolean(properties.getProperty("turret.isEnabled"));
    magazineEnabled = Boolean.parseBoolean(properties.getProperty("magazine.isEnabled"));
    climberEnabled = Boolean.parseBoolean(properties.getProperty("climber.isEnabled"));
    compressorEnabled = Boolean.parseBoolean(properties.getProperty("compressor.isEnabled"));

    drivetrainLogLevel = Level.OFF;
    intakeLogLevel = Level.OFF;
    shooterLogLevel = Level.OFF;
    spinnerLogLevel = Level.OFF;
    queueLogLevel = Level.OFF;
    turretLogLevel = Level.OFF;
    magazineLogLevel = Level.OFF;
    climberLogLevel = Level.OFF;
    compressorLogLevel = Level.OFF;
    limelightLogLevel = Level.OFF;

    if (limelightEnabled) {
      m_limelight = new Limelight(Pipeline.valueOf(properties.getProperty("limelight.defaultPipeline")));
      limelightLogLevel = Level.parse(properties.getProperty("limelight.logLevel"));
    }
    if (turretEnabled && drivetrainEnabled && limelightEnabled) {m_hexagonPosition = new HexagonPosition(m_drivetrain, m_turret, m_limelight);}

    if (drivetrainEnabled) {
      m_drivetrain = new Drivetrain();
      drivetrainLogLevel = Level.parse(properties.getProperty("drivetrain.logLevel"));
      m_defaultDriveCommand = Commands.simpleArcadeDrive(m_drivetrain, () -> getMove(), () -> getTurn());
      m_drivetrain.setDefaultCommand(m_defaultDriveCommand);
    }

    if (intakeEnabled) {
      m_intake = new Intake();
      intakeLogLevel = Level.parse(properties.getProperty("intake.logLevel"));
    }

    if (shooterEnabled) {
      m_shooter = new Shooter();
      shooterLogLevel = Level.parse(properties.getProperty("shooter.logLevel"));
    }

    if (spinnerEnabled) {
      m_spinner = new Spinner();
      spinnerLogLevel = Level.parse(properties.getProperty("spinner.logLevel"));
    }

    if (queueEnabled) {
      m_queue = new Queue();
      queueLogLevel = Level.parse(properties.getProperty("queue.logLevel"));
    }

    if (turretEnabled) {
      m_turret = new Turret();
      turretLogLevel = Level.parse(properties.getProperty("turret.logLevel"));
    }

    if (magazineEnabled) {
      m_magazine = new Magazine();
      magazineLogLevel = Level.parse(properties.getProperty("magazine.logLevel"));
      m_magazine.setDefaultCommand(Commands.autoRefillQueue(m_magazine, 0.5, () -> m_queue.getBallInQueue()));
    }

    if (climberEnabled) {
      m_climber = new Climber();
      climberLogLevel = Level.parse(properties.getProperty("climber.logLevel"));
    }

    if (compressorEnabled) {
      m_compressor = new CompressorManager();
      compressorLogLevel = Level.parse(properties.getProperty("compressor.logLevel"));
    }
    
    compressorLog.setLevel(compressorLogLevel);
    climberLog.setLevel(climberLogLevel);
    magazineLog.setLevel(magazineLogLevel);
    turretLog.setLevel(turretLogLevel);
    queueLog.setLevel(queueLogLevel);
    spinnerLog.setLevel(spinnerLogLevel);
    shooterLog.setLevel(shooterLogLevel);
    intakeLog.setLevel(intakeLogLevel);
    drivetrainLog.setLevel(drivetrainLogLevel);
    limelightLog.setLevel(limelightLogLevel);

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
    if (drivetrainEnabled) m_drivetrain.saveProperties();
    if (intakeEnabled) m_intake.savePID();
    if (shooterEnabled) m_shooter.savePID();
    if (turretEnabled) m_turret.savePID();
  }
}
