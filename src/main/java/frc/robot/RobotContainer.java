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
import java.io.File;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick.ButtonType;
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
import frc.robot.vision.PiTable;
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
  private final MagazinePowerCellCounter m_magazinePowerCellCounter;
  private final HexagonPosition m_hexagonPosition;
  private final Limelight m_limelight;
  private final PiTable m_piTable;
  private final IDrivetrain m_drivetrain;
  private final IIntake m_intake;
  private final IShooter m_shooter;
  private final ISpinner m_spinner;
  private final IQueue m_queue;
  private final ITurret m_turret;
  private final IMagazine m_magazine;
  private final IClimber m_climber;
  private final ICompressorManager m_compressor;
  private final Properties m_properties;

  private final PowerDistributionPanel pdp = new PowerDistributionPanel();

  private String m_pathPath = "paths/auto1.wpilib.json";

  private final boolean m_limelightEnabled;
  private final boolean m_piTableEnabled;
  private final boolean m_compressorEnabled;
  private final boolean m_drivetrainEnabled;
  private final boolean m_intakeEnabled;
  private final boolean m_shooterEnabled;
  private final boolean m_spinnerEnabled;
  private final boolean m_queueEnabled;
  private final boolean m_turretEnabled;
  private final boolean m_magazineEnabled;
  private final boolean m_climberEnabled;

  private final Command m_defaultDriveCommand;

  private Command m_autonomousCommand;

  private final double m_intakeExtenderSpeed = 0.3;
  private final double m_intakeSpeed = 0.7;
  private final double m_magazineSpeed = 0.5;
  private final double m_queueSpeed = 0.75;
  private final double m_shooterHoodSpeed = 0.4;
  private final double m_turretSpeed = 0.3;

  private static Logger logger = Logger.getGlobal();
  private static Logger commandLogger = Logger.getLogger(Commands.class.getName());

  private final Map<ButtonCommand, Button> m_buttonMap = new HashMap<>();
  private final Map<JoystickCommand, DoubleSupplier> m_joystickMap = new HashMap<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_controlBoard = new ControlBoard();

    m_buttonMap.put(ButtonCommand.TOGGLE_COMPRESSOR, m_controlBoard.buttonBox.bigRed);

    m_buttonMap.put(ButtonCommand.SHIFT_DRIVETRAIN, m_controlBoard.xbox.rightBumper);

    m_buttonMap.put(ButtonCommand.EXTEND_INTAKE, m_controlBoard.xbox.aButton);
    m_buttonMap.put(ButtonCommand.RETRACT_INTAKE, m_controlBoard.xbox.yButton);

    m_buttonMap.put(ButtonCommand.MANUAL_RUN_INTAKE, m_controlBoard.extreme.baseFrontLeft);
    m_buttonMap.put(ButtonCommand.MANUAL_RUN_INTAKE_REVERSE, m_controlBoard.extreme.baseFrontRight);
    m_buttonMap.put(ButtonCommand.INTAKE_EXTENDER_MOTION_MAGIC, m_controlBoard.extreme.sideButton);
    // m_buttonMap.put(ButtonCommand.INTAKE, );

    m_buttonMap.put(ButtonCommand.MANUAL_RUN_MAGAZINE, m_controlBoard.extreme.baseMiddleRight);
    m_buttonMap.put(ButtonCommand.MANUAL_RUN_MAGAZINE_REVERSE, m_controlBoard.extreme.baseMiddleLeft);

    m_buttonMap.put(ButtonCommand.MANUAL_RUN_QUEUE, m_controlBoard.extreme.baseBackLeft);
    m_buttonMap.put(ButtonCommand.MANUAL_RUN_QUEUE_REVERSE, m_controlBoard.extreme.baseBackRight);

    m_buttonMap.put(ButtonCommand.MOVE_POWER_CELLS, m_controlBoard.buttonBox.yellow);
    m_buttonMap.put(ButtonCommand.MOVE_POWER_CELLS_REVERSE, m_controlBoard.buttonBox.green);
    m_buttonMap.put(ButtonCommand.AUTO_REFILL_QUEUE, m_controlBoard.buttonBox.bottomRed);

    m_buttonMap.put(ButtonCommand.MANUAL_RUN_SHOOTER, m_controlBoard.extreme.trigger);
    m_buttonMap.put(ButtonCommand.AUTO_RUN_SHOOTER, m_controlBoard.buttonBox.bottomWhite);
    m_buttonMap.put(ButtonCommand.AUTO_SHOOT, m_controlBoard.buttonBox.middleRed);

    m_buttonMap.put(ButtonCommand.AUTO_AIM_TURRET, m_controlBoard.buttonBox.bigWhite);
    m_buttonMap.put(ButtonCommand.STOP_MOTORS, m_controlBoard.buttonBox.topRed);

    m_buttonMap.put(ButtonCommand.TURRET_TESTING_MOTION_MAGIC, m_controlBoard.extreme.baseFrontLeft);

    
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
    m_joystickMap.put(JoystickCommand.MANUAL_RUN_SHOOTER, () -> 0.0);
    m_joystickMap.put(JoystickCommand.INTAKE_IN, () -> m_controlBoard.xbox.getRightTrigger());
    m_joystickMap.put(JoystickCommand.INTAKE_OUT, () -> m_controlBoard.xbox.getLeftTrigger());
    m_joystickMap.put(JoystickCommand.MANUAL_RUN_SHOOTER_HOOD, () -> m_controlBoard.extreme.getPOVY() * m_shooterHoodSpeed);
    m_joystickMap.put(JoystickCommand.MANUAL_MOVE_TURRET, () -> m_controlBoard.extreme.getTwist() * m_turretSpeed);

    m_properties = PropertyFiles.loadProperties(RobotContainer.class.getSimpleName().toLowerCase());

    File robotMapPropertiesFile = new File(Filesystem.getDeployDirectory() + "/RobotMap.properties");
    Properties robotMapProperties = PropertyFiles.loadProperties(robotMapPropertiesFile);

    m_limelightEnabled = Boolean.parseBoolean(m_properties.getProperty("limelight.isEnabled"));
    m_piTableEnabled = Boolean.parseBoolean(m_properties.getProperty("piTable.isEnabled"));

    m_compressorEnabled = Boolean.parseBoolean(m_properties.getProperty("compressor.isEnabled"));
    m_drivetrainEnabled = Boolean.parseBoolean(m_properties.getProperty("drivetrain.isEnabled"));
    m_intakeEnabled = Boolean.parseBoolean(m_properties.getProperty("intake.isEnabled"));
    m_magazineEnabled = Boolean.parseBoolean(m_properties.getProperty("magazine.isEnabled"));
    m_queueEnabled = Boolean.parseBoolean(m_properties.getProperty("queue.isEnabled"));
    m_shooterEnabled = Boolean.parseBoolean(m_properties.getProperty("shooter.isEnabled"));
    m_turretEnabled = Boolean.parseBoolean(m_properties.getProperty("turret.isEnabled"));
    m_climberEnabled = Boolean.parseBoolean(m_properties.getProperty("climber.isEnabled"));
    m_spinnerEnabled = Boolean.parseBoolean(m_properties.getProperty("spinner.isEnabled"));

    logger.setLevel(Level.parse(m_properties.getProperty("globalLogLevel").toUpperCase()));
    commandLogger.setLevel(Level.parse(m_properties.getProperty("commandsLogLevel").toUpperCase()));

    // File path to generated robot path
    m_pathPath = m_properties.getProperty("PATH_PATH");

    SmartDashboard.putBoolean("limelight enabled", m_limelightEnabled);

    m_piTable = m_piTableEnabled ? new PiTable() : null;

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

    m_limelight = m_limelightEnabled ? new Limelight(Pipeline.valueOf(m_properties.getProperty("limelight.defaultPipeline"))) : null;
    m_compressor = m_compressorEnabled ? new CompressorManager() : new DummyCompressor();
    m_drivetrain = m_drivetrainEnabled ? new Drivetrain(drivetrainMap) : new DummyDrivetrain();
    m_intake = m_intakeEnabled ? new Intake(intakeMap) : new DummyIntake();
    m_shooter = m_shooterEnabled ? new Shooter(shooterMap) : new DummyShooter();
    m_spinner = m_spinnerEnabled ? new Spinner() : new DummySpinner();
    m_magazine = m_magazineEnabled ? new Magazine(magazineMap) : new DummyMagazine();
    m_queue = m_queueEnabled ? new Queue(queueMap) : new DummyQueue();
    m_turret = m_turretEnabled ? new Turret(turretMap) : new DummyTurret();
    m_climber = m_climberEnabled ? new Climber(climberMap) : new DummyClimber();
    //not dead code
    if (m_turretEnabled && m_drivetrainEnabled && m_limelightEnabled) {
      logger.log(Level.INFO, "initalized hexagon position");
      m_hexagonPosition = new HexagonPosition(m_drivetrain, m_turret, m_limelight);
    } else {m_hexagonPosition = null;}
    m_magazinePowerCellCounter = new MagazinePowerCellCounter(m_magazine.getPhotoEye(), m_queue.getPhotoEye(), m_magazine);

    m_defaultDriveCommand = Commands.simpleArcadeDrive(m_drivetrain, m_joystickMap.get(JoystickCommand.MOVE), m_joystickMap.get(JoystickCommand.TURN));
    
    m_drivetrain.setDefaultCommand(Commands.simpleArcadeDrive(m_drivetrain, m_joystickMap.get(JoystickCommand.MOVE), m_joystickMap.get(JoystickCommand.TURN)));
    // m_intake.setDefaultCommand(Commands.runIntakeExtender_Temp(m_intake, m_joystickMap.get(JoystickCommand.MANUAL_RUN_INTAKE_EXTENDER)));
    m_shooter.setDefaultCommand(Commands.runShooter(m_shooter, m_joystickMap.get(JoystickCommand.MANUAL_RUN_SHOOTER)));
    m_turret.setDefaultCommand(Commands.moveTurret(m_turret, m_joystickMap.get(JoystickCommand.MANUAL_MOVE_TURRET)));
    m_intake.setDefaultCommand(Commands.intakeCommand(m_intake, this::getIntakeSpeed, m_intakeSpeed, m_magazine,
        m_magazineSpeed, m_joystickMap.get(JoystickCommand.MANUAL_RUN_INTAKE_EXTENDER)::getAsDouble,
        m_intakeExtenderSpeed, this::getIntakeExtended, this::runExtenderMotionMagic));

    configureButtonBindings();

    m_autonomousCommand = m_drivetrainEnabled ? Commands.followPath(m_drivetrain, m_pathPath) : null;
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Toggle limelight LED force off
    m_buttonMap.get(ButtonCommand.SHIFT_DRIVETRAIN)
      .whenPressed(Commands.drivetrainSetLowGear(m_drivetrain, true))
      .whenReleased(Commands.drivetrainSetLowGear(m_drivetrain, false));
    // m_buttonMap.get(ButtonCommand.INVERT_DRIVING)
    //   .whenPressed(Commands.setDrivingInverted(m_drivetrain, true))
    //   .whenReleased(Commands.setDrivingInverted(m_drivetrain, false));

    // Toggle intake extender motion magic
    // m_buttonMap.get(ButtonCommand.INTAKE_EXTENDER_MOTION_MAGIC).whileHeld(Commands.toggleIntakeExtended(m_intake));

    // Toggle intake extended
    // m_buttonMap.get(ButtonCommand.EXTEND_INTAKE).whenPressed(Commands.setIntakeExtended(m_intake, true));
    // m_buttonMap.get(ButtonCommand.RETRACT_INTAKE).whenPressed(Commands.setIntakeExtended(m_intake, false));

    // Start/stop intaking
    // m_buttonMap.get(ButtonCommand.MANUAL_RUN_INTAKE_REVERSE).whileHeld(Commands.runIntake(m_intake, -m_intakeSpeed));

    m_buttonMap.get(ButtonCommand.MANUAL_RUN_MAGAZINE).whileHeld(Commands.runMagazine(m_magazine, m_magazineSpeed));
    m_buttonMap.get(ButtonCommand.MANUAL_RUN_MAGAZINE_REVERSE).whileHeld(Commands.runMagazine(m_magazine, -m_magazineSpeed));
    
    m_buttonMap.get(ButtonCommand.MANUAL_RUN_QUEUE).whileHeld(Commands.runQueue(m_queue, m_queueSpeed));
    m_buttonMap.get(ButtonCommand.MANUAL_RUN_QUEUE_REVERSE).whileHeld(Commands.runQueue(m_queue, -m_queueSpeed));

    m_buttonMap.get(ButtonCommand.MOVE_POWER_CELLS).whileHeld(
      Commands.runMagazine(m_magazine, m_magazineSpeed)
      .alongWith(Commands.runQueue(m_queue, m_queueSpeed)));
    m_buttonMap.get(ButtonCommand.MOVE_POWER_CELLS_REVERSE).whileHeld(
      Commands.runIntake(m_intake, -m_intakeSpeed)
      .alongWith(Commands.runMagazine(m_magazine, -m_magazineSpeed)
      .alongWith(Commands.runQueue(m_queue, -m_queueSpeed))));
    // Add auto refill qeuue
    
    m_buttonMap.get(ButtonCommand.MANUAL_RUN_SHOOTER).whileHeld(Commands.setShooterVelocity(m_shooter, m_shooter::getCalculatedVelocity));
    m_buttonMap.get(ButtonCommand.AUTO_RUN_SHOOTER).toggleWhenPressed(Commands.setShooterVelocity(m_shooter, m_shooter::getCalculatedVelocity));
    m_buttonMap.get(ButtonCommand.STOP_MOTORS).toggleWhenPressed(Commands.stopMotors(m_magazine, m_queue, m_shooter));

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
    if (m_limelightEnabled) m_buttonMap.get(ButtonCommand.AUTO_AIM_TURRET).toggleWhenPressed(Commands.findTarget(m_turret));

    m_buttonMap.get(ButtonCommand.TURRET_TESTING_MOTION_MAGIC).whileHeld(Commands.runTurretPosition(m_turret, 0.0));
    m_buttonMap.get(ButtonCommand.TOGGLE_COMPRESSOR).whenPressed(Commands.toggleCompressor(m_compressor));
  }

  public void setDriveType(DriveType driveType) {
    if (m_drivetrainEnabled) {
      IDrivetrain drivetrain = m_drivetrain;
      Command driveCommand;
      switch (driveType) {
        case SIMPLE_ARCADE_DRIVE:
          driveCommand = Commands.simpleArcadeDrive(drivetrain, m_joystickMap.get(JoystickCommand.MOVE), m_joystickMap.get(JoystickCommand.TURN));
          break;
        case VELOCITY_ARCADE_DRIVE:
          driveCommand = Commands.velocityArcadeDrive(drivetrain, m_joystickMap.get(JoystickCommand.MOVE), m_joystickMap.get(JoystickCommand.TURN));
          break;
        case ACCELERATION_LIMITED_SIMPLE_ARCADE_DRIVE:
          driveCommand = Commands.accelerationLimitedSimpleArcadeDrive(drivetrain, m_joystickMap.get(JoystickCommand.MOVE), m_joystickMap.get(JoystickCommand.TURN), 2, 3);
          break;
        case ACCELERATION_LIMITED_VELOCITY_ARCADE_DRIVE:
          driveCommand = Commands.accelerationLimitedVelocityArcadeDrive(drivetrain, m_joystickMap.get(JoystickCommand.MOVE), m_joystickMap.get(JoystickCommand.TURN), 2, 3);
          break;
        default:
          driveCommand = m_defaultDriveCommand;
          break;
      }
      drivetrain.setDefaultCommand(driveCommand);
    }
  }

  public double getIntakeSpeed() {
    double inTrigger = m_joystickMap.get(JoystickCommand.INTAKE_IN).getAsDouble();
    double outTrigger = -m_joystickMap.get(JoystickCommand.INTAKE_OUT).getAsDouble();
    SmartDashboard.putNumber("intake in trigger", inTrigger);
    SmartDashboard.putNumber("intake out trigger", outTrigger);
    if (inTrigger > 0.0) {
      return inTrigger;
    } else if(outTrigger < 0.0) {
      return outTrigger;
    } else {
      return 0.0;
    }
  }

  public boolean getIntakeExtended() {
    boolean extendButton = m_buttonMap.get(ButtonCommand.EXTEND_INTAKE).get();
    boolean retractButton = m_buttonMap.get(ButtonCommand.RETRACT_INTAKE).get();
    boolean extended = false;
    if (extendButton) extended = extendButton;
    else if(retractButton) extended = !retractButton;
    return extended;
  }

  public boolean runExtenderMotionMagic() {
    boolean toggleButtonState = false;
    boolean toggleButton = m_buttonMap.get(ButtonCommand.INTAKE_EXTENDER_MOTION_MAGIC).get();
    boolean motionMagic = false;
    if (toggleButton && toggleButtonState == false) {motionMagic = !motionMagic;}
    toggleButtonState = toggleButton;
    return motionMagic;
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
    SmartDashboard.putNumber("power cell count", MagazinePowerCellCounter.getCount());
    if (m_hexagonPosition != null) m_hexagonPosition.updatePosition();
    if (m_limelightEnabled) SmartDashboard.putNumber("distance", m_limelight.getDistanceToTarget());
  }

  /**
   * saves all the properties in properties files in subsystems. should be called in disabled init and any other time nessescary.
   * <p> <B> <I> subsystesm must run their property saving methods here for them to save unless theyre called elswhere
   */
  public void saveAllProperties() {
    if (m_drivetrainEnabled) m_drivetrain.saveProperties();
    if (m_intakeEnabled) m_intake.saveProperties();
    if (m_shooterEnabled) m_shooter.saveProperties();
    if (m_turretEnabled) m_turret.saveProperties();
  }
}
