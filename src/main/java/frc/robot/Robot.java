/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Properties;
import java.util.logging.Level;
import java.util.logging.Logger;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.Commands;
import frc.robot.controlboard.ControlBoard;
import frc.robot.sensors.ColorSensor.ColorDetect;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CompressorManager;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Queue;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spinner;
import frc.robot.subsystems.Turret;
import frc.robot.utils.DareMathUtil;
import frc.robot.utils.DriveType;
import frc.robot.utils.MagazinePowerCellCounter;
import frc.robot.utils.PropertyFiles;
import frc.robot.vision.HexagonPosition;
import frc.robot.vision.Limelight;
import frc.robot.vision.Limelight.Pipeline;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private final SendableChooser<DriveType> m_driveTypeChooser = new SendableChooser<>();
  private DriveType m_currentDriveType = null;

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private ControlBoard m_controlBoard;
  private MagazinePowerCellCounter m_magazinePowerCellCounter = new MagazinePowerCellCounter();
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
  private Properties properties;

  private String m_pathPath = "paths/auto1.wpilib.json";

  private static Logger logger = Logger.getLogger(Robot.class.getName());

  private static Logger climberLog = Logger.getLogger(Climber.class.getName());
  private static Logger compressorLog = Logger.getLogger(CompressorManager.class.getName());
  private static Logger drivetrainLog = Logger.getLogger(Drivetrain.class.getName());
  private static Logger intakeLog = Logger.getLogger(Intake.class.getName());
  private static Logger magazineLog = Logger.getLogger(Magazine.class.getName());
  private static Logger queueLog = Logger.getLogger(Queue.class.getName());
  private static Logger shooterLog = Logger.getLogger(Shooter.class.getName());
  private static Logger spinnerLog = Logger.getLogger(Spinner.class.getName());
  private static Logger turretLog = Logger.getLogger(Turret.class.getName());
  private static Logger limelightLog = Logger.getLogger(Limelight.class.getName());

  private boolean limelightEnabled;
  private boolean drivetrainEnabled;
  private boolean intakeEnabled;
  private boolean shooterEnabled;
  private boolean spinnerEnabled;
  private boolean queueEnabled;
  private boolean turretEnabled;
  private boolean magazineEnabled;
  private boolean climberEnabled;
  private boolean compressorEnabled;

  private Command m_defaultDriveCommand;

  private boolean m_autoRefillQueueEnabled = false;
  private boolean m_autoFeedShooterEnabled = false;

  private double m_intakeExtenderSpeed = 0.2;
  private double m_magazineSpeed = 0.5;
  private double m_queueSpeed = 0.5;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    m_driveTypeChooser.setDefaultOption("Simple arcade drive", DriveType.SIMPLE_ARCADE_DRIVE);
    m_driveTypeChooser.addOption("Velocity arcade drive", DriveType.VELOCITY_ARCADE_DRIVE);
    m_driveTypeChooser.addOption("Acceleration limited simple arcade drive", DriveType.ACCELERATION_LIMITED_SIMPLE_ARCADE_DRIVE);
    m_driveTypeChooser.addOption("Acceleration limited velocity arcade drive", DriveType.ACCELERATION_LIMITED_VELOCITY_ARCADE_DRIVE);
    SmartDashboard.putData("Drive type", m_driveTypeChooser);
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

    limelightLog.setLevel(Level.parse(properties.getProperty("limelight.logLevel").toUpperCase()));
    drivetrainLog.setLevel(Level.parse(properties.getProperty("drivetrain.logLevel").toUpperCase()));
    intakeLog.setLevel(Level.parse(properties.getProperty("intake.logLevel").toUpperCase()));
    shooterLog.setLevel(Level.parse(properties.getProperty("shooter.logLevel").toUpperCase()));
    spinnerLog.setLevel(Level.parse(properties.getProperty("spinner.logLevel").toUpperCase()));
    magazineLog.setLevel(Level.parse(properties.getProperty("magazine.logLevel").toUpperCase()));
    queueLog.setLevel(Level.parse(properties.getProperty("queue.logLevel").toUpperCase()));
    turretLog.setLevel(Level.parse(properties.getProperty("turret.logLevel").toUpperCase()));
    climberLog.setLevel(Level.parse(properties.getProperty("climber.logLevel").toUpperCase()));
    compressorLog.setLevel(Level.parse(properties.getProperty("compressor.logLevel").toUpperCase()));

    SmartDashboard.putBoolean("limelight enabled", limelightEnabled);
    SmartDashboard.putBoolean("drivetrain enabled", drivetrainEnabled);
    SmartDashboard.putBoolean("intake enabled", intakeEnabled);
    SmartDashboard.putBoolean("shooter enabled", shooterEnabled);
    SmartDashboard.putBoolean("spinner enabled", spinnerEnabled);
    SmartDashboard.putBoolean("queue enabled", queueEnabled);
    SmartDashboard.putBoolean("turret enabled", turretEnabled);
    SmartDashboard.putBoolean("magazine enabled", magazineEnabled);
    SmartDashboard.putBoolean("climber enabled", climberEnabled);
    SmartDashboard.putBoolean("compressor enabled", compressorEnabled);

    if (limelightEnabled) {
      m_limelight = new Limelight(Pipeline.valueOf(properties.getProperty("limelight.defaultPipeline")));
    }

    if (turretEnabled && drivetrainEnabled && limelightEnabled) {
      m_hexagonPosition = new HexagonPosition(m_drivetrain, m_turret, m_limelight);
    }

    if (drivetrainEnabled) {
      m_drivetrain = new Drivetrain();
      m_defaultDriveCommand = Commands.simpleArcadeDrive(m_drivetrain, this::getMove, this::getTurn);
      m_drivetrain.setDefaultCommand(m_defaultDriveCommand);
    }

    if (intakeEnabled) {
      m_intake = new Intake();
      m_intake.setDefaultCommand(Commands.runIntakeExtender_Temp(m_intake, this::getIntakeExtenderSpeed));
    }

    if (shooterEnabled) {
      m_shooter = new Shooter();
      SmartDashboard.putNumber("desired shooter speed", getShooterSpeed());
      m_shooter.setDefaultCommand(new RunCommand(() -> m_shooter.setPercentOutput(getShooterSpeed()), m_shooter));
    }

    if (spinnerEnabled) {
      m_spinner = new Spinner();
    }
    if (magazineEnabled) {
      m_magazine = new Magazine(m_magazinePowerCellCounter::incrementCount, m_magazinePowerCellCounter::decrementCount);
      m_magazine.setDefaultCommand(Commands.runMagazine(m_magazine, this::getMagazineSpeed));
    }

    if (queueEnabled) {
      m_queue = new Queue(m_magazinePowerCellCounter::incrementCount, m_magazinePowerCellCounter::decrementCount);
      m_queue.setDefaultCommand(Commands.runQueue(m_queue, this::getQueueSpeed));
    }

    if (turretEnabled) {
      m_turret = new Turret();
    }

    if (climberEnabled) {
      m_climber = new Climber();
    }

    if (compressorEnabled) {
      m_compressor = new CompressorManager();
    }

    configureButtonBindings();

    m_autonomousCommand = drivetrainEnabled ? Commands.followPath(m_drivetrain, m_pathPath) : null;
    logger.config("robot initialized");
  }

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
      m_controlBoard.getButton("runIntake").toggleWhenPressed(Commands.runIntake(m_intake, 0.5));
      // Toggle intake extender motion magic
      m_controlBoard.getButton("toggleIntakeMagic").whenPressed(new InstantCommand(() -> m_intake.toggleMotionMagicEnabled(), m_intake));
      m_controlBoard.getButton("toggleIntake").whenPressed(new InstantCommand(() -> m_intake.toggleExtended(), m_intake));
    }

    if (magazineEnabled && queueEnabled) {
      // Toggle auto queue refilling
      m_controlBoard.getButton("autoRefillQueue").whenPressed(new InstantCommand(() -> {
        m_autoRefillQueueEnabled = !m_autoRefillQueueEnabled;
        if (m_autoRefillQueueEnabled) {
          m_magazine.setDefaultCommand(Commands.autoRefillQueue(m_magazine, m_magazineSpeed, m_queue::hasPowerCell));
        } else {
          m_magazine.setDefaultCommand(Commands.runMagazine(m_magazine, this::getMagazineSpeed));
        }
      }));
    }

    if (queueEnabled && shooterEnabled) {
      m_controlBoard.getButton("autoFeedShooter").whenPressed(new InstantCommand(() -> {
        m_autoFeedShooterEnabled = !m_autoFeedShooterEnabled;
        if (m_autoFeedShooterEnabled) {
          m_queue.setDefaultCommand(Commands.autoFeedShooter(m_queue, m_queueSpeed, m_magazinePowerCellCounter::getCount));
        } else {
          m_queue.setDefaultCommand(Commands.runQueue(m_queue, this::getQueueSpeed));
        }
      }));
    }

    if (queueEnabled) {
      m_controlBoard.getButton("runQueue").whileHeld(Commands.runQueue(m_queue, 1.0));
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
    logger.config("configured buttons");
  }

  private double getMove() {
    double speed = m_controlBoard.xbox.getLeftStickY();
    return speed;
  }

  private double getTurn() {
    double speed = m_controlBoard.xbox.getRightStickX();
    return speed;
  }

  private double getIntakeExtenderSpeed() {
    double speed = m_controlBoard.extreme.getStickY();
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
    double speed = -m_controlBoard.extreme.getSlider();
    speed = DareMathUtil.mapRange(speed, -1, 1, 0, 1);
    SmartDashboard.putNumber("Slider", speed);
    return speed;
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
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    DriveType newDriveType = m_driveTypeChooser.getSelected();
    if (newDriveType != m_currentDriveType) {
      setDriveType(newDriveType);
    }
    SmartDashboard.putNumber("power cell count", m_magazinePowerCellCounter.getCount());

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    saveAllProperties();
  }

  public void saveAllProperties() {
    if (drivetrainEnabled) m_drivetrain.saveProperties();
    if (intakeEnabled) m_intake.saveProperties();
    if (shooterEnabled) m_shooter.saveProperties();
    if (turretEnabled) m_turret.saveProperties();
    logger.config("finished saving properties");
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand.schedule();
    logger.config("schecduled auton");
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    logger.config("intitialized teleop");
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
