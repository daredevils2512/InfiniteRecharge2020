/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.HashMap;
import java.util.Map;
import java.util.Properties;
import java.util.function.DoubleSupplier;
import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;

import frc.robot.commands.Commands;
import frc.robot.controlboard.ButtonCommand;
import frc.robot.controlboard.ControlBoard;
import frc.robot.controlboard.JoystickCommand;
import frc.robot.controlboard.JoystickUtil;
import frc.robot.subsystems.dummy.DummyClimber;
import frc.robot.subsystems.dummy.DummyCompressor;
import frc.robot.subsystems.dummy.DummyDrivetrain;
import frc.robot.subsystems.dummy.DummyIntake;
import frc.robot.subsystems.dummy.DummyMagazine;
import frc.robot.subsystems.dummy.DummyQueue;
import frc.robot.subsystems.dummy.DummyShooter;
import frc.robot.subsystems.dummy.DummySpinner;
import frc.robot.subsystems.dummy.DummyTurret;
import frc.robot.subsystems.interfaces.IClimber;
import frc.robot.subsystems.interfaces.ICompressorManager;
import frc.robot.subsystems.interfaces.IDrivetrain;
import frc.robot.subsystems.interfaces.IIntake;
import frc.robot.subsystems.interfaces.IMagazine;
import frc.robot.subsystems.interfaces.IQueue;
import frc.robot.subsystems.interfaces.IShooter;
import frc.robot.subsystems.interfaces.ISpinner;
import frc.robot.subsystems.interfaces.ITurret;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CompressorManager;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Queue;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spinner;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Climber.ClimberMap;
import frc.robot.subsystems.Drivetrain.DrivetrainMap;
import frc.robot.subsystems.Intake.IntakeMap;
import frc.robot.subsystems.Magazine.MagazineMap;
import frc.robot.subsystems.Queue.QueueMap;
import frc.robot.subsystems.Shooter.ShooterMap;
import frc.robot.subsystems.Turret.TurretMap;
import frc.robot.utils.BoundedCounter;
import frc.robot.utils.DriveType;
import frc.robot.utils.PropertyFiles;
import frc.robot.vision.HexagonPosition;
import frc.robot.vision.Limelight;
import frc.robot.vision.Limelight.Pipeline;

import edu.wpi.first.wpilibj.Counter;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  //somewhat evil singleton counter
  private static Counter m_powerCellCounter = new Counter();

  public static Counter getCounter(){
    return m_powerCellCounter;
  }
  //gloriously evil

  private final ControlBoard m_controlBoard;
  private HexagonPosition m_hexagonPosition;
  private Limelight m_limelight;
  private ICompressorManager m_compressor;
  private IDrivetrain m_drivetrain;
  private IIntake m_intake;
  private IMagazine m_magazine;
  private IQueue m_queue;
  private IShooter m_shooter;
  private ITurret m_turret;
  private IClimber m_climber;
  private ISpinner m_spinner;

  private BoundedCounter m_magazinePowerCellCounter = new BoundedCounter(0, 3);
  

  private final Properties properties;

  private String m_pathPath = "paths/auto1.wpilib.json";

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
  private Command m_manualIntakeCommand;
  private Command m_manualMagazineCommand;
  private Command m_manualQueueCommand;

  private Command m_autonomousCommand;

  private boolean m_intakeRunning = false;
  private boolean m_magazineRunning = false;
  private boolean m_queueRunning = false;
  private boolean m_autoRefillQueueEnabled = false;
  private boolean m_autoFeedShooterEnabled = false;

  private double m_intakeExtenderSpeed = 0.3;
  private double m_magazineSpeed = 0.5;
  private double m_queueSpeed = 0.5;

  private final Map<ButtonCommand, Button> m_buttonMap = new HashMap<>();
  private final Map<JoystickCommand, DoubleSupplier> m_joystickMap = new HashMap<>();

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_controlBoard = new ControlBoard();

    m_buttonMap.put(ButtonCommand.INVERT_DRIVING, m_controlBoard.xbox.leftTrigger);
    m_buttonMap.put(ButtonCommand.SHIFT_DRIVETRAIN, m_controlBoard.xbox.rightTrigger);
    m_buttonMap.put(ButtonCommand.MANUAL_RUN_INTAKE, m_controlBoard.xbox.aButton);
    m_buttonMap.put(ButtonCommand.MANUAL_RUN_MAGAZINE, m_controlBoard.extreme.joystickTopRight);
    m_buttonMap.put(ButtonCommand.MANUAL_RUN_QUEUE, m_controlBoard.extreme.joystickBottomRight);

    m_joystickMap.put(JoystickCommand.MOVE, () -> {
      double move = -m_controlBoard.xbox.getLeftStickY();
      move = JoystickUtil.deadband(move, 0.05);
      move = Math.abs(Math.pow(move, 2)) * Math.signum(move);
      return move;
    });
    m_joystickMap.put(JoystickCommand.TURN, () -> {
      double turn = -m_controlBoard.xbox.getRightStickX();
      turn = JoystickUtil.deadband(turn, 0.05);
      turn = Math.abs(Math.pow(turn, 2)) * Math.signum(turn);
      return turn;
    });
    m_joystickMap.put(JoystickCommand.MANUAL_RUN_INTAKE_EXTENDER, () -> -m_controlBoard.extreme.getStickY());
    m_joystickMap.put(JoystickCommand.MANUAL_RUN_SHOOTER, () -> m_controlBoard.extreme.getSlider());
    m_joystickMap.put(JoystickCommand.MANUAL_MOVE_TURRET, () -> m_controlBoard.extreme.getPOVX());

    // This should probably be extracted from here and from PropertySubsystem at some point
    properties = PropertyFiles.loadProperties(RobotContainer.class.getSimpleName().toLowerCase());

    String robotMapPropertiesFilename = RobotContainer.class.getSimpleName() + ".properties";
    File robotMapDefaultPropertiesFile = new File(Filesystem.getOperatingDirectory() + "/" + robotMapPropertiesFilename);
    File robotMapPropertiesFile = new File(Filesystem.getDeployDirectory() + "/" + robotMapPropertiesFilename);
    Properties robotMapProperties = PropertyFiles.loadProperties(robotMapDefaultPropertiesFile, robotMapPropertiesFile);

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

    if (limelightEnabled) {
      m_limelight = new Limelight(Pipeline.valueOf(properties.getProperty("limelight.defaultPipeline")));
    }

    if (turretEnabled && drivetrainEnabled && limelightEnabled) {
      m_hexagonPosition = new HexagonPosition(m_drivetrain, m_turret, m_limelight);
    }

    DrivetrainMap drivetrainMap = new Drivetrain.DrivetrainMap();
    drivetrainMap.driveLeft1ID = Integer.parseInt(robotMapProperties.getProperty("driveLeft1ID"));
    drivetrainMap.driveLeft2ID = Integer.parseInt(robotMapProperties.getProperty("driveLeft2ID"));
    drivetrainMap.driveRight1ID = Integer.parseInt(robotMapProperties.getProperty("driveRight1ID"));
    drivetrainMap.driveRight2ID = Integer.parseInt(robotMapProperties.getProperty("driveRight2ID"));
    drivetrainMap.pigeonID = Integer.parseInt(robotMapProperties.getProperty("pigeonID"));
    drivetrainMap.driveLeftEncoderChannelA = Integer.parseInt(robotMapProperties.getProperty("driveLeftEncoderChannelA"));
    drivetrainMap.driveLeftEncoderChannelB = Integer.parseInt(robotMapProperties.getProperty("driveLeftEncoderChannelB"));
    drivetrainMap.driveRightEncoderChannelA = Integer.parseInt(robotMapProperties.getProperty("driveRightEncoderChannelA"));
    drivetrainMap.driveRightEncoderChannelB = Integer.parseInt(robotMapProperties.getProperty("driveRightEncoderChannelB"));
    drivetrainMap.shiftForwardChannel = Integer.parseInt(robotMapProperties.getProperty("drivetrainShiftForwardChannel"));
    drivetrainMap.shiftReverseChannel = Integer.parseInt(robotMapProperties.getProperty("drivetrainShiftReverseChannel"));

    IntakeMap intakeMap = new IntakeMap();
    intakeMap.runMotorID = Integer.parseInt(robotMapProperties.getProperty("intakeRunID"));
    intakeMap.extendMotorID = Integer.parseInt(robotMapProperties.getProperty("intakeExtenderID"));
    intakeMap.retractedLimitSwitchChannel = Integer.parseInt(robotMapProperties.getProperty("intakeRetractedLimitSwitchChannel"));
    intakeMap.extendedLimitSwitchChannel = Integer.parseInt(robotMapProperties.getProperty("intakeExtendedLimitSwitchChannel"));

    ShooterMap shooterMap = new ShooterMap();
    shooterMap.shooter1ID = Integer.parseInt(robotMapProperties.getProperty("shooter1ID"));
    shooterMap.shooter2ID = Integer.parseInt(robotMapProperties.getProperty("shooter2ID"));
    shooterMap.shooterHoodID = Integer.parseInt(robotMapProperties.getProperty("shooterHoodID"));
    
    MagazineMap magazineMap = new MagazineMap();
    magazineMap.runMotorID = Integer.parseInt(robotMapProperties.getProperty("magazineRunID"));
    magazineMap.photoEyeChannel = Integer.parseInt(robotMapProperties.getProperty("magazinePhotoEyeChannel"));

    QueueMap queueMap = new QueueMap();
    queueMap.queueRunID = Integer.parseInt(robotMapProperties.getProperty("queueRunID"));
    queueMap.photoEyeChannel = Integer.parseInt(robotMapProperties.getProperty("queuePhotoEyeChannel"));

    TurretMap turretMap = new TurretMap();
    turretMap.turretID = Integer.parseInt(robotMapProperties.getProperty("turretID"));

    ClimberMap climberMap = new ClimberMap();
    climberMap.climberLeftID = Integer.parseInt(robotMapProperties.getProperty("climberLeftID"));
    climberMap.climberRightID = Integer.parseInt(robotMapProperties.getProperty("climberRightID"));

    m_drivetrain = drivetrainEnabled ? new Drivetrain(drivetrainMap) : new DummyDrivetrain();
    m_intake = intakeEnabled ? new Intake(intakeMap) : new DummyIntake();
    m_shooter = shooterEnabled ? new Shooter(shooterMap) : new DummyShooter();
    m_spinner = spinnerEnabled ? new Spinner() : new DummySpinner();
    m_magazine = magazineEnabled ? new Magazine(magazineMap) : new DummyMagazine();
    m_queue = queueEnabled ? new Queue(queueMap) : new DummyQueue();
    m_turret = turretEnabled ? new Turret(turretMap) : new DummyTurret();
    m_climber = climberEnabled ? new Climber(climberMap) : new DummyClimber();
    m_compressor = compressorEnabled ? new CompressorManager() : new DummyCompressor();


/*
    m_magazine.onPowerCellIn(() -> m_magazinePowerCellCounter.increment());
    m_magazine.onPowerCellOut(() -> m_magazinePowerCellCounter.decrement());
    m_queue.onPowerCellInMagazine(() -> m_magazinePowerCellCounter.increment());
    m_queue.onPowerCellOutMagazine(() -> m_magazinePowerCellCounter.decrement());
*/
    /*
      NOW wherever you run code that switches the motor directions, you switch the up down sources
      OR you create a second counter that inverts the up/down source and then you substract the 2 counter values to get a current value.. (maybe)
      i'd test just using teh coutner
    */
    

    m_drivetrain.setDefaultCommand(Commands.simpleArcadeDrive(m_drivetrain, m_joystickMap.get(JoystickCommand.MOVE), m_joystickMap.get(JoystickCommand.TURN)));
    m_intake.setDefaultCommand(m_manualIntakeCommand);
    m_magazine.setDefaultCommand(m_manualMagazineCommand);
    m_queue.setDefaultCommand(m_manualQueueCommand);
    m_shooter.setDefaultCommand(Commands.runShooter(m_shooter, m_joystickMap.get(JoystickCommand.MANUAL_RUN_SHOOTER)));

    m_climber.setLogLevel(properties.getProperty("climber.logLevel"));
    m_intake.setLogLevel(properties.getProperty("intake.logLevel"));
    m_shooter.setLogLevel(properties.getProperty("shooter.logLevel"));
    m_drivetrain.setLogLevel(properties.getProperty("drivetrain.logLevel"));
    m_spinner.setLogLevel(properties.getProperty("spinner.logLevel"));
    m_magazine.setLogLevel(properties.getProperty("magazine.logLevel"));
    m_queue.setLogLevel(properties.getProperty("queue.logLevel"));
    m_turret.setLogLevel(properties.getProperty("turret.logLevel"));
    m_compressor.setLogLevel(properties.getProperty("compressor.logLevel"));

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
      m_buttonMap.get(ButtonCommand.SHIFT_DRIVETRAIN)
        .whenPressed(Commands.drivetrainSetLowGear(m_drivetrain, true))
        .whenReleased(Commands.drivetrainSetLowGear(m_drivetrain, false));
      m_buttonMap.get(ButtonCommand.INVERT_DRIVING)
        .whenPressed(Commands.setDrivingInverted(m_drivetrain, true))
        .whenReleased(Commands.setDrivingInverted(m_drivetrain, false));

      // Toggle intake extender motion magic
      m_buttonMap.get(ButtonCommand.INTAKE_EXTENDER_MOTION_MAGIC).whenPressed(new InstantCommand(() -> m_intake.toggleMotionMagicEnabled(), m_intake));
      // Toggle intake extended
      m_buttonMap.get(ButtonCommand.EXTEND_INTAKE).whenPressed(new InstantCommand(() -> m_intake.toggleExtended(), m_intake));
      // Start/stop intaking
      m_buttonMap.get(ButtonCommand.MANUAL_RUN_INTAKE).whenPressed(() -> m_intakeRunning = !m_intakeRunning);

      // Toggle between having the magazine automatically refilling the queue
      // and having the magazine be run manually
      m_buttonMap.get(ButtonCommand.AUTO_REFILL_QUEUE).whenPressed(new InstantCommand(() -> {
        m_autoRefillQueueEnabled = !m_autoRefillQueueEnabled;
        if (m_autoRefillQueueEnabled) {
          m_magazine.setDefaultCommand(Commands.autoRefillQueue(m_magazine, m_magazineSpeed, m_queue::hasPowerCell));
        } else {
          m_magazine.setDefaultCommand(m_manualMagazineCommand);
        }
      }));

      // Toggle between having the queue automatically feed the shooter
      // (which should check if the shooter and turret are ready to shoot)
      // and having the queue be manually run
      m_buttonMap.get(ButtonCommand.AUTO_FEED_SHOOTER).whenPressed(new InstantCommand(() -> {
        m_autoFeedShooterEnabled = !m_autoFeedShooterEnabled;
        if (m_autoFeedShooterEnabled) {
          m_queue.setDefaultCommand(Commands.autoFeedShooter(m_queue, m_queueSpeed, m_magazinePowerCellCounter::getCount));
        } else {
          m_queue.setDefaultCommand(m_manualQueueCommand);
        }
      }));

      // Toggle between having the turret automatically track the target
      // and having the turret be turned manually
      m_buttonMap.get(ButtonCommand.AUTO_AIM_TURRET).toggleWhenPressed(Commands.findTarget(m_turret, m_limelight, 5));

    if (shooterEnabled) {
      // Run shooter at a set motor output
      // m_controlBoard.extreme.sideButton.whileHeld(Commands.runShooter(m_shooter, () -> 0.5));
    }
    
    
    if (spinnerEnabled) {
      // Extend/retract spinner
      // m_controlBoard.getButton("extendSpinner").whenPressed(Commands.setSpinnerExtended(m_spinner, true));
      // m_controlBoard.getButton("retractSpinner").whenPressed(Commands.setSpinnerExtended(m_spinner, false));

      // m_controlBoard.getButton("spinnerRotationControl").whenPressed(Commands.rotationControl(m_spinner, 3));
      // m_controlBoard.getButton("spinnerColorControl").whenPressed(Commands.precisionControl(m_spinner, ColorDetect.Red));
    }
  }

  public void setDriveType(DriveType driveType) {
    if (drivetrainEnabled) {
      Command driveCommand;
      switch (driveType) {
        case SIMPLE_ARCADE_DRIVE:
          driveCommand = Commands.simpleArcadeDrive(m_drivetrain, m_joystickMap.get(JoystickCommand.MOVE), m_joystickMap.get(JoystickCommand.TURN));
          break;
        case VELOCITY_ARCADE_DRIVE:
          driveCommand = Commands.velocityArcadeDrive(m_drivetrain, m_joystickMap.get(JoystickCommand.MOVE), m_joystickMap.get(JoystickCommand.TURN));
          break;
        case ACCELERATION_LIMITED_SIMPLE_ARCADE_DRIVE:
          driveCommand = Commands.accelerationLimitedSimpleArcadeDrive(m_drivetrain, m_joystickMap.get(JoystickCommand.MOVE), m_joystickMap.get(JoystickCommand.TURN), 2, 3);
          break;
        case ACCELERATION_LIMITED_VELOCITY_ARCADE_DRIVE:
          driveCommand = Commands.accelerationLimitedVelocityArcadeDrive(m_drivetrain, m_joystickMap.get(JoystickCommand.MOVE), m_joystickMap.get(JoystickCommand.TURN), 2, 3);
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
