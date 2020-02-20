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
import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;
import java.util.Properties;
import java.util.function.Consumer;
import java.util.logging.*;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Commands;
import frc.robot.controlboard.ControlBoard;
import frc.robot.controlboard.JoystickUtil;
import frc.robot.controlboard.Xbox;
import frc.robot.sensors.ColorSensor.ColorDetect;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Queue;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spinner;
import frc.robot.subsystems.*;
import frc.robot.utils.DareMathUtil;
import frc.robot.utils.DriveType;
import frc.robot.utils.PropertyFiles;
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
  private String m_pathPath = "paths/auto1.wpilib.json";

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

  private Command m_autonomousCommand;

  private boolean m_autoRefillQueueEnabled = false;
  private boolean m_autoFeedShooterEnabled = false;

  private double m_intakeExtenderSpeed = 0.2;
  private double m_magazineSpeed = 0.5;
  private double m_queueSpeed = 0.5;

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_controlBoard = new ControlBoard();


    properties = PropertyFiles.loadProperties(RobotContainer.class.getSimpleName().toLowerCase());

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
    // File path to generated robot path
    m_pathPath = properties.getProperty("PATH_PATH");


    limelightLog.setLevel(Level.OFF);
    drivetrainLog.setLevel(Level.OFF);
    intakeLog.setLevel(Level.OFF);
    shooterLog.setLevel(Level.OFF);
    spinnerLog.setLevel(Level.OFF);
    queueLog.setLevel(Level.OFF);
    turretLog.setLevel(Level.OFF);
    magazineLog.setLevel(Level.OFF);
    climberLog.setLevel(Level.OFF);
    compressorLog.setLevel(Level.OFF);

    if (limelightEnabled) {
      limelightLog.setLevel(Level.parse(properties.getProperty("limelight.logLevel")));
      m_limelight = new Limelight(Pipeline.valueOf(properties.getProperty("limelight.defaultPipeline")));
    }

    if (turretEnabled && drivetrainEnabled && limelightEnabled) {
      m_hexagonPosition = new HexagonPosition(m_drivetrain, m_turret, m_limelight);
    }

    if (drivetrainEnabled) {
      drivetrainLog.setLevel(Level.parse(properties.getProperty("drivetrain.logLevel")));
      m_drivetrain = new Drivetrain();
      m_defaultDriveCommand = Commands.simpleArcadeDrive(m_drivetrain, () -> getMove(), () -> getTurn());
      m_drivetrain.setDefaultCommand(m_defaultDriveCommand);
    }

    if (intakeEnabled) {
      intakeLog.setLevel(Level.parse(properties.getProperty("intake.logLevel")));
      m_intake = new Intake();
      m_intake.setDefaultCommand(Commands.runIntakeExtender_Temp(m_intake, () -> getIntakeExtenderSpeed()));
    }

    if (shooterEnabled) {
      shooterLog.setLevel(Level.parse(properties.getProperty("shooter.logLevel")));
      m_shooter = new Shooter();
      m_shooter.setDefaultCommand(new RunCommand(() -> m_shooter.setPercentOutput(0.5), m_shooter));
    }

    if (spinnerEnabled) {
      spinnerLog.setLevel(Level.parse(properties.getProperty("spinner.logLevel")));
      m_spinner = new Spinner();
    }
    if (magazineEnabled) {
      magazineLog.setLevel(Level.parse(properties.getProperty("magazine.logLevel")));
      m_magazine = new Magazine();
      m_magazine.setDefaultCommand(Commands.runMagazine(m_magazine, () -> getMagazineSpeed()));
    }

    if (queueEnabled) {
      queueLog.setLevel(Level.parse(properties.getProperty("queue.logLevel")));
      m_queue = new Queue();
      m_queue.setDefaultCommand(Commands.runQueue(m_queue, () -> getQueueSpeed()));
    }

    if (turretEnabled) {
      turretLog.setLevel(Level.parse(properties.getProperty("turret.logLevel")));
      m_turret = new Turret();
    }

    if (climberEnabled) {
      climberLog.setLevel(Level.parse(properties.getProperty("climber.logLevel")));
      m_climber = new Climber();
    }

    if (compressorEnabled) {
      compressorLog.setLevel(Level.parse(properties.getProperty("compressor.logLevel")));
      m_compressor = new CompressorManager();
    }

    configureButtonBindings();

    m_autonomousCommand = drivetrainEnabled ? Commands.followPath(m_drivetrain, m_pathPath) : null;
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
      m_controlBoard.getButton("shiftDown")
        .whenPressed(Commands.drivetrainSetLowGear(m_drivetrain, true))
        .whenReleased(Commands.drivetrainSetLowGear(m_drivetrain, false));
      m_controlBoard.getButton("invertDriving")
        .whenPressed(Commands.setDrivingInverted(m_drivetrain, true))
        .whenReleased(Commands.setDrivingInverted(m_drivetrain, false));
    }

    if (intakeEnabled && magazineEnabled) {
      // Start/stop intaking
      m_controlBoard.getButton("runIntake").toggleWhenPressed(Commands.runIntake(m_intake, m_magazine, 1));
    }

    if (magazineEnabled && queueEnabled) {
      // Toggle auto queue refilling
      m_controlBoard.getButton("autoRefillQueue").whenPressed(new InstantCommand(() -> {
        m_autoRefillQueueEnabled = !m_autoRefillQueueEnabled;
        if (m_autoRefillQueueEnabled) {
          m_magazine.setDefaultCommand(Commands.autoRefillQueue(m_magazine, m_magazineSpeed, () -> m_queue.hasPowerCell()));
        } else {
          m_magazine.setDefaultCommand(Commands.runMagazine(m_magazine, () -> getMagazineSpeed()));
        }
      }));
    }

    if (queueEnabled && shooterEnabled) {
      m_controlBoard.getButton("autoFeedShooter").whenPressed(new InstantCommand(() -> {
        m_autoFeedShooterEnabled = !m_autoFeedShooterEnabled;
        if (m_autoFeedShooterEnabled) {
          m_queue.setDefaultCommand(Commands.autoFeedShooter(m_queue, m_queueSpeed, () -> m_magazine.getPowerCellCount()));
        } else {
          m_queue.setDefaultCommand(Commands.runQueue(m_queue, () -> getQueueSpeed()));
        }
      }));
    }

    if (turretEnabled && limelightEnabled) {
      m_controlBoard.getButton("toggleFindTarget").toggleWhenPressed(Commands.findTarget(m_turret, m_limelight, 5));
    }

    if (shooterEnabled) {
      // Run shooter at a set motor output
      // m_controlBoard.extreme.sideButton.whileHeld(Commands.runShooter(m_shooter, () -> 0.5));
    }
    
    if (spinnerEnabled) {
      // Extend/retract spinner
      m_controlBoard.getButton("extendSpinner").whenPressed(Commands.setSpinnerExtended(m_spinner, true));
      m_controlBoard.getButton("retractSpinner").whenPressed(Commands.setSpinnerExtended(m_spinner, false));

      m_controlBoard.getButton("spinnerRotationControl").whenPressed(Commands.rotationControl(m_spinner, 3));
      m_controlBoard.getButton("spinnerColorControl").whenPressed(Commands.precisionControl(m_spinner, ColorDetect.Red));
    }

    if (turretEnabled) {
      m_turret.setDefaultCommand(Commands.moveTurret(m_turret, m_controlBoard.extreme::getPOVX));
    }
  }

  private double getMove() {
    double move = -m_controlBoard.xbox.getLeftStickY();
    move = JoystickUtil.deadband(move, 0.05);
    move = Math.abs(Math.pow(move, 2)) * Math.signum(move);
    return move / 2;
  }

  private double getTurn() {
    double turn = -m_controlBoard.xbox.getRightStickX();
    turn = JoystickUtil.deadband(turn, 0.05);
    turn = Math.abs(Math.pow(turn, 2)) * Math.signum(turn);
    return turn / 2;
  }

  /**
   * Temporary function for testing the intake
   * @return
   */
  private double getIntakeExtenderSpeed() {
    double speed = m_controlBoard.extreme.getPOVY();
    return speed * m_intakeExtenderSpeed;
  }

  /**
   * Manual magazine control
   * @return Scaled magazine speed
   */
  private double getMagazineSpeed() {
    double speed = m_controlBoard.extreme.joystickTopRight.get() ? m_magazineSpeed : 0;
    return speed;
  }

  /**
   * Manual queue control
   * @return Scaled queue speed
   */
  private double getQueueSpeed() {
    double speed = m_controlBoard.extreme.joystickBottomRight.get() ? m_queueSpeed : 0;
    return speed;
  }

  private double getShooterSpeed() {
    double speed = m_controlBoard.extreme.getSlider();
    speed = DareMathUtil.mapRange(speed, -1, 1, 0, 1);
    return 1.0;
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
    // return drivetrainEnabled ? Commands.followPath(m_drivetrain, "test.wpilib.json") : null;
  }

  /**
   * saves all the properties in properties files in subsystems. should be called in disabled init and any other time nessescary.
   * <p> <B> <I> subsystesm must run their property saving methods here for them to save unless theyre called elswhere
   */
  public void saveAllProperties() {
    if (drivetrainEnabled) m_drivetrain.saveProperties();
    if (intakeEnabled) m_intake.saveProperties();
    if (shooterEnabled) m_shooter.saveProperties();
    if (turretEnabled) m_turret.saveProperties();
  }
}
