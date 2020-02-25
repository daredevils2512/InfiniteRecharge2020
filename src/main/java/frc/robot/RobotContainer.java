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
import java.util.logging.Level;
import java.util.logging.Logger;
import java.lang.reflect.Field;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.Commands;
import frc.robot.controlboard.ButtonCommand;
import frc.robot.controlboard.ControlBoard;
import frc.robot.controlboard.JoystickCommand;
import frc.robot.controlboard.JoystickUtil;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Climber.ClimberMap;
import frc.robot.subsystems.Drivetrain.DrivetrainMap;
import frc.robot.subsystems.Intake.IntakeMap;
import frc.robot.subsystems.Magazine.MagazineMap;
import frc.robot.subsystems.Queue.QueueMap;
import frc.robot.subsystems.Shooter.ShooterMap;
import frc.robot.subsystems.Turret.TurretMap;
import frc.robot.subsystems.dummy.*;
import frc.robot.subsystems.interfaces.*;
import frc.robot.utils.DriveType;
import frc.robot.utils.MagazinePowerCellCounter;
import frc.robot.utils.PropertyFiles;
import frc.robot.vision.HexagonPosition;
import frc.robot.vision.Limelight;
import frc.robot.vision.Limelight.Pipeline;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final ControlBoard m_controlBoard;
  private MagazinePowerCellCounter m_magazinePowerCellCounter;
  private HexagonPosition m_hexagonPosition;
  private Limelight m_limelight;
  private IDrivetrain m_drivetrain = new DummyDrivetrain();
  private IIntake m_intake = new DummyIntake();
  private IShooter m_shooter = new DummyShooter();
  private ISpinner m_spinner = new DummySpinner();
  private IQueue m_queue = new DummyQueue();
  private ITurret m_turret = new DummyTurret();
  private IMagazine m_magazine = new DummyMagazine();
  private IClimber m_climber = new DummyClimber();
  private ICompressorManager m_compressor = new DummyCompressor();
  private final Properties properties;

  private String m_pathPath = "paths/auto1.wpilib.json";

  private final boolean limelightEnabled;
  private boolean drivetrainEnabled = false;
  private boolean intakeEnabled = false;
  private boolean shooterEnabled = false;
  private boolean spinnerEnabled = false;
  private boolean queueEnabled = false;
  private boolean turretEnabled = false;
  private boolean magazineEnabled = false;
  private boolean climberEnabled = false;
  private boolean compressorEnabled = false;

  private final Command m_defaultDriveCommand;

  private Command m_autonomousCommand;

  private final double m_intakeExtenderSpeed = 0.3;
  private final double m_intakeSpeed = 0.5;
  private final double m_magazineSpeed = 0.5;
  private final double m_queueSpeed = 0.75;
  private final double m_shooterHoodSpeed = 0.5;
  private final double m_turretSpeed = 0.5;

  private static Logger logger = Logger.getGlobal();

  private ILogging[] subsystemArray;

  private final Map<ButtonCommand, Button> m_buttonMap = new HashMap<>();
  private final Map<JoystickCommand, DoubleSupplier> m_joystickMap = new HashMap<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_controlBoard = new ControlBoard();

    subsystemArray = new ILogging[] {m_drivetrain, m_intake, m_turret, m_magazine, m_queue, m_shooter, m_spinner, m_climber, m_compressor};

    m_buttonMap.put(ButtonCommand.REVERSE_INTAKE, m_controlBoard.xbox.leftBumper);
    m_buttonMap.put(ButtonCommand.SHIFT_DRIVETRAIN, m_controlBoard.xbox.rightBumper);
    m_buttonMap.put(ButtonCommand.AUTO_RUN_SHOOTER, m_controlBoard.xbox.xButton);
    m_buttonMap.put(ButtonCommand.MANUAL_RUN_INTAKE, m_controlBoard.xbox.aButton);
    m_buttonMap.put(ButtonCommand.MANUAL_RUN_MAGAZINE, m_controlBoard.extreme.joystickTopRight);
    m_buttonMap.put(ButtonCommand.MANUAL_REVERSE_MAGAZINE, m_controlBoard.extreme.joystickBottomRight);
    m_buttonMap.put(ButtonCommand.HOOD_UP, m_controlBoard.extreme.joystickTopLeft);
    m_buttonMap.put(ButtonCommand.HOOD_DOWN, m_controlBoard.extreme.joystickBottomLeft);
    // m_buttonMap.put(ButtonCommand.MANUAL_RUN_QUEUE, m_controlBoard.extreme.joystickTopLeft);
    // m_buttonMap.put(ButtonCommand.MANUAL_REVERSE_QUEUE, m_controlBoard.extreme.joystickBottomLeft);
    m_buttonMap.put(ButtonCommand.INTAKE_EXTENDER_MOTION_MAGIC, m_controlBoard.extreme.sideButton);
    m_buttonMap.put(ButtonCommand.TURRET_TESTING_MOTION_MAGIC, m_controlBoard.extreme.baseFrontRight);
    m_buttonMap.put(ButtonCommand.SHOOT, m_controlBoard.extreme.trigger);
    // m_buttonMap.put(ButtonCommand.EXTEND_INTAKE, m_controlBoard.xbox.aButton);
    // m_buttonMap.put(ButtonCommand.AUTO_REFILL_QUEUE, m_controlBoard.extreme.joystickTopLeft);
    // m_buttonMap.put(ButtonCommand.AUTO_FEED_SHOOTER, m_controlBoard.extreme.joystickBottomLeft);
    m_buttonMap.put(ButtonCommand.AUTO_AIM_TURRET, m_controlBoard.extreme.trigger);

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
    m_joystickMap.put(JoystickCommand.MANUAL_RUN_INTAKE_EXTENDER, () -> -m_controlBoard.extreme.getStickY() * m_intakeExtenderSpeed);
    m_joystickMap.put(JoystickCommand.MANUAL_RUN_SHOOTER, () -> m_controlBoard.xbox.getRightTrigger());
    m_joystickMap.put(JoystickCommand.MANUAL_RUN_SHOOTER_HOOD, () -> m_controlBoard.extreme.getPOVY() * m_shooterHoodSpeed);
    m_joystickMap.put(JoystickCommand.MANUAL_MOVE_TURRET, () -> m_controlBoard.extreme.getPOVX() * m_turretSpeed);

    // This should probably be extracted from here and from PropertySubsystem at
    // some point
    properties = PropertyFiles.loadProperties(RobotContainer.class.getSimpleName().toLowerCase());

    // String robotMapPropertiesFilename = RobotContainer.class.getSimpleName() + ".properties";
    // File robotMapDefaultPropertiesFile = new File(
    //     Filesystem.getOperatingDirectory() + "/" + robotMapPropertiesFilename);
    // File robotMapPropertiesFile = new File(Filesystem.getDeployDirectory() + "/" + robotMapPropertiesFilename);
    Properties robotMapProperties = PropertyFiles.loadProperties("RobotMap");

    for (ILogging subsystem : subsystemArray) {
      String dummyName = subsystem.getClass().getSimpleName();
      String name = dummyName.toLowerCase().split("y")[1];
      logger.config("initializing" + name);
      try {
        Field isEnabled = this.getClass().getDeclaredField(name + "Enabled");
        isEnabled.setAccessible(true);
        isEnabled.set(this, Boolean.parseBoolean(properties.getProperty(name + ".isEnabled")));
        SmartDashboard.putBoolean(name + " enabled", isEnabled.getBoolean(this));
      } catch (NoSuchFieldException | SecurityException | IllegalArgumentException | IllegalAccessException e) {
        logger.log(Level.SEVERE, name + " failed to load; threw : ", e);
      }
    }

    logger.setLevel(Level.parse(properties.getProperty("globalLogLevel").toUpperCase()));

    limelightEnabled = Boolean.parseBoolean(properties.getProperty("limelight.isEnabled"));
    // File path to generated robot path
    m_pathPath = properties.getProperty("PATH_PATH");

    SmartDashboard.putBoolean("limelight enabled", limelightEnabled);

    if (limelightEnabled) {
      m_limelight = new Limelight(Pipeline.valueOf(properties.getProperty("limelight.defaultPipeline")));
    }

    //not dead code
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

    m_magazinePowerCellCounter = new MagazinePowerCellCounter(m_magazine.getPhotoEye(), m_queue.getPhotoEye(), m_magazine);

    m_defaultDriveCommand = Commands.simpleArcadeDrive(m_drivetrain, m_joystickMap.get(JoystickCommand.MOVE), m_joystickMap.get(JoystickCommand.TURN));
    m_shooter.setDefaultCommand(Commands.runShooter(m_shooter, this::getShooterSpeed));

    m_drivetrain.setDefaultCommand(Commands.simpleArcadeDrive(m_drivetrain, m_joystickMap.get(JoystickCommand.MOVE), m_joystickMap.get(JoystickCommand.TURN)));
    m_intake.setDefaultCommand(Commands.runIntakeExtender_Temp(m_intake, m_controlBoard.extreme::getStickY));
    
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
      // m_buttonMap.get(ButtonCommand.INVERT_DRIVING)
      //   .whenPressed(Commands.setDrivingInverted(m_drivetrain, true))
      //   .whenReleased(Commands.setDrivingInverted(m_drivetrain, false));

      // Toggle intake extender motion magic
      m_buttonMap.get(ButtonCommand.INTAKE_EXTENDER_MOTION_MAGIC).whileHeld(Commands.toggleIntakeExtended(m_intake));
      // Toggle intake extended
      // m_buttonMap.get(ButtonCommand.EXTEND_INTAKE).whenPressed();
      // Start/stop intaking
      m_buttonMap.get(ButtonCommand.MANUAL_RUN_INTAKE).whileHeld(Commands.intakeBall(m_intake, m_intakeSpeed, m_magazine, m_magazineSpeed, m_magazinePowerCellCounter));
      m_buttonMap.get(ButtonCommand.REVERSE_INTAKE).whileHeld(Commands.runIntake(m_intake, -m_intakeSpeed));
      m_buttonMap.get(ButtonCommand.MANUAL_RUN_MAGAZINE).toggleWhenPressed(Commands.runMagazine(m_magazine, m_magazineSpeed));
      m_buttonMap.get(ButtonCommand.MANUAL_REVERSE_MAGAZINE).toggleWhenPressed(Commands.runMagazine(m_magazine, -m_magazineSpeed)).whileHeld(Commands.runIntake(m_intake, -m_intakeSpeed));
      // m_buttonMap.get(ButtonCommand.MANUAL_RUN_MAGAZINE).whenReleased(Commands.runMagazine(m_magazine, 0.0));
      // m_buttonMap.get(ButtonCommand.MANUAL_RUN_QUEUE).toggleWhenPressed(Commands.runQueue(m_queue, m_queueSpeed));
      // m_buttonMap.get(ButtonCommand.MANUAL_REVERSE_QUEUE).toggleWhenPressed(Commands.runQueue(m_queue, -m_queueSpeed));
      // m_buttonMap.get(ButtonCommand.MANUAL_RUN_QUEUE).whenReleased(Commands.runQueue(m_queue, 0.0));
      m_buttonMap.get(ButtonCommand.AUTO_RUN_SHOOTER).whileHeld(Commands.setShooterVelocity(m_shooter, () -> 5000.0));

      m_buttonMap.get(ButtonCommand.HOOD_UP).whileHeld(Commands.runHood(m_shooter, () -> m_shooterHoodSpeed));
      m_buttonMap.get(ButtonCommand.HOOD_DOWN).whileHeld(Commands.runHood(m_shooter, () -> -m_shooterHoodSpeed));


      m_buttonMap.get(ButtonCommand.SHOOT).whileHeld(Commands.runMagazine(m_magazine, m_magazineSpeed))
        .whileHeld(Commands.runIntake(m_intake, m_intakeSpeed))
        .whileHeld(Commands.runQueue(m_queue, m_queueSpeed))
        .whileHeld(Commands.runShooter(m_shooter, () -> 9000));

      m_buttonMap.get(ButtonCommand.SHIFT_DRIVETRAIN).whenPressed(Commands.drivetrainToggleLowGear(m_drivetrain));
      // Toggle between having the queue automatically feed the shooter
      // (which should check if the shooter and turret are ready to shoot)
      // and having the queue be manually run //idk how well this will work
      // m_buttonMap.get(ButtonCommand.AUTO_FEED_SHOOTER).whenPressed(new InstantCommand(() -> {
      //   m_autoFeedShooterEnabled = !m_autoFeedShooterEnabled;
      //   if (m_autoFeedShooterEnabled) {
      //     m_queue.setDefaultCommand(Commands.autoFeedShooter(m_queue, m_queueSpeed, m_magazinePowerCellCounter::getCount));
      //   } else {
      //     // m_queue.setDefaultCommand(m_manualQueueCommand);
      //   }
      // }));

      // Toggle between having the turret automatically track the target
      // and having the turret be turned manually
      // m_buttonMap.get(ButtonCommand.AUTO_AIM_TURRET).toggleWhenPressed(Commands.findTarget(m_turret, m_limelight, 5));

      m_buttonMap.get(ButtonCommand.TURRET_TESTING_MOTION_MAGIC).whileHeld(Commands.runTurretPosition(m_turret, 0.0));

      m_turret.setDefaultCommand(Commands.moveTurret(m_turret, () -> m_controlBoard.extreme.getTwist() * 0.3));
  }

  private double getShooterSpeed() {
    return m_controlBoard.xbox.getRightTrigger() * 9000;
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

  //beacuse
  public void robotContainerPeriodic() {
    m_magazinePowerCellCounter.updateCount();
    SmartDashboard.putNumber("power cell count", m_magazinePowerCellCounter.getCount());
    if (m_hexagonPosition != null) SmartDashboard.putBoolean("can shoot", m_hexagonPosition.canShoot());
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
