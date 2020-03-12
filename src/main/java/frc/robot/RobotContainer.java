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

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;

import frc.robot.commands.Commands;
import frc.robot.commands.ShootBalls;
import frc.robot.controlboard.ButtonCommand;
import frc.robot.controlboard.ControlBoard;
import frc.robot.controlboard.JoystickCommand;
import frc.robot.controlboard.JoystickUtil;
import frc.robot.subsystems.*;
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
  private final NetworkTable m_networkTable;
  private final NetworkTableEntry m_climberEntry;
  private final NetworkTableEntry m_compressorEntry;
  private final NetworkTableEntry m_drivetrainEntry;
  private final NetworkTableEntry m_intakeEntry;
  private final NetworkTableEntry m_magazineEntry;
  private final NetworkTableEntry m_queueEntry;
  private final NetworkTableEntry m_shooterEntry;
  private final NetworkTableEntry m_spinnerEntry;
  private final NetworkTableEntry m_turretEntry;

  private final ControlBoard m_controlBoard;
  private final MagazinePowerCellCounter m_magazinePowerCellCounter;
  private final HexagonPosition m_hexagonPosition;
  private final Limelight m_limelight;
  // private final PiTable m_piTable;
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
  private final double m_magazineSpeed = 0.8;
  private final double m_queueSpeed = 0.9;
  private final double m_shooterHoodSpeed = 0.4;
  private final double m_turretSpeed = 0.3;

  private static Logger logger = Logger.getGlobal();
  private static Logger commandLogger = Logger.getLogger(Commands.class.getName());
  private static Logger limelightLogger = Logger.getLogger(Limelight.class.getName());

  private final Map<ButtonCommand, Button> m_buttonMap = new HashMap<>();
  private final Map<JoystickCommand, DoubleSupplier> m_joystickMap = new HashMap<>();

  private boolean extenderMotionMagicButtonState;
  private boolean motionMagic;
  private boolean intakeExtended;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_controlBoard = new ControlBoard();

    m_networkTable = NetworkTableInstance.getDefault().getTable(RobotContainer.class.getName());

    m_climberEntry = m_networkTable.getEntry("climber is used");
    m_compressorEntry = m_networkTable.getEntry("compressor is used");
    m_drivetrainEntry = m_networkTable.getEntry("drivetrain is used");
    m_intakeEntry = m_networkTable.getEntry("intake is used");
    m_magazineEntry = m_networkTable.getEntry("magazine is used");
    m_queueEntry = m_networkTable.getEntry("queue is used");
    m_shooterEntry = m_networkTable.getEntry("shooter is used");
    m_spinnerEntry = m_networkTable.getEntry("spinner is used");
    m_turretEntry = m_networkTable.getEntry("turret is used");

    putButtons();

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

    m_limelight = m_limelightEnabled ? new Limelight(Pipeline.valueOf(m_properties.getProperty("limelight.defaultPipeline"))) : null;
    m_compressor = m_compressorEnabled ? new CompressorManager() : new DummyCompressor();
    m_drivetrain = m_drivetrainEnabled ? new Drivetrain(robotMapProperties) : new DummyDrivetrain();
    m_intake = m_intakeEnabled ? new Intake(robotMapProperties) : new DummyIntake();
    m_shooter = m_shooterEnabled ? new Shooter(robotMapProperties) : new DummyShooter();
    m_spinner = m_spinnerEnabled ? new Spinner() : new DummySpinner();
    m_magazine = m_magazineEnabled ? new Magazine(robotMapProperties) : new DummyMagazine();
    m_queue = m_queueEnabled ? new Queue(robotMapProperties) : new DummyQueue();
    m_turret = m_turretEnabled ? new Turret(robotMapProperties) : new DummyTurret();
    m_climber = m_climberEnabled ? new Climber(robotMapProperties) : new DummyClimber();
    //not dead code
    if (m_turretEnabled && m_drivetrainEnabled && m_limelightEnabled) {
      logger.log(Level.INFO, "initalized hexagon position");
      m_hexagonPosition = new HexagonPosition(m_drivetrain, m_turret, m_limelight);
    } else {m_hexagonPosition = null;}
    // m_magazinePowerCellCounter = new MagazinePowerCellCounter(m_magazine.getPhotoEye(), m_queue.getPhotoEye(), m_magazine);
    m_magazinePowerCellCounter = new MagazinePowerCellCounter(m_queue, m_magazine);

    m_defaultDriveCommand = Commands.simpleArcadeDrive(m_drivetrain, m_joystickMap.get(JoystickCommand.MOVE), m_joystickMap.get(JoystickCommand.TURN));
    
    m_drivetrain.setDefaultCommand(Commands.simpleArcadeDrive(m_drivetrain, m_joystickMap.get(JoystickCommand.MOVE), m_joystickMap.get(JoystickCommand.TURN)));
    // m_intake.setDefaultCommand(Commands.runIntakeExtender_Temp(m_intake, m_joystickMap.get(JoystickCommand.MANUAL_RUN_INTAKE_EXTENDER)));
    m_shooter.setDefaultCommand(Commands.setShooterVelocity(m_shooter, () -> -500.0));
    // m_shooter.setDefaultCommand(Commands.runShooter(m_shooter, () -> 0.0));
    m_turret.setDefaultCommand(Commands.moveTurret(m_turret, m_joystickMap.get(JoystickCommand.MANUAL_MOVE_TURRET)));
    m_intake.setDefaultCommand(Commands.intakeCommand(m_intake, this::getIntakeSpeed, m_intakeSpeed, m_magazine,
        m_magazineSpeed, m_joystickMap.get(JoystickCommand.MANUAL_RUN_INTAKE_EXTENDER)::getAsDouble,
        m_intakeExtenderSpeed, this::getIntakeExtended, this::runExtenderMotionMagic));
    
    m_climber.setLogLevel(m_properties.getProperty("climber.logLevel"));
    m_intake.setLogLevel(m_properties.getProperty("intake.logLevel"));
    m_shooter.setLogLevel(m_properties.getProperty("shooter.logLevel"));
    m_spinner.setLogLevel(m_properties.getProperty("spinner.logLevel"));
    m_queue.setLogLevel(m_properties.getProperty("queue.logLevel"));
    m_compressor.setLogLevel(m_properties.getProperty("compressor.logLevel"));
    m_drivetrain.setLogLevel(m_properties.getProperty("drivetrain.logLevel"));
    m_magazine.setLogLevel(m_properties.getProperty("magazine.logLevel"));
    m_turret.setLogLevel(m_properties.getProperty("turret.logLevel"));
    // limelightLogger.setLevel(Level.parse(m_properties.getProperty("limelight.logLevel")));

    configureButtonBindings();

    // m_autonomousCommand = m_drivetrainEnabled ? Commands.followPath(m_drivetrain, m_pathPath) : null;
    m_autonomousCommand = Commands.autoCommand(m_shooter, m_queue, m_queueSpeed, m_turret, m_limelight, m_magazine, m_magazineSpeed, MagazinePowerCellCounter.getCount(), m_drivetrain, -1.0);
  }

  private void putButtons() {














    //important buttons

    m_buttonMap.put(ButtonCommand.MANUAL_RUN_INTAKE, m_controlBoard.buttonBox.topWhite);
    m_buttonMap.put(ButtonCommand.MANUAL_RUN_INTAKE_REVERSE, m_controlBoard.buttonBox.topRed);


    m_buttonMap.put(ButtonCommand.MANUAL_RUN_MAGAZINE, m_controlBoard.buttonBox.middleWhite);
    m_buttonMap.put(ButtonCommand.MANUAL_RUN_MAGAZINE_REVERSE, m_controlBoard.buttonBox.middleRed);
    
    
    m_buttonMap.put(ButtonCommand.MANUAL_RUN_QUEUE, m_controlBoard.buttonBox.bottomWhite);
    m_buttonMap.put(ButtonCommand.MANUAL_RUN_QUEUE_REVERSE, m_controlBoard.buttonBox.bottomRed);

    m_buttonMap.put(ButtonCommand.SHOOT_BALL, m_controlBoard.buttonBox.bigWhite);


    m_buttonMap.put(ButtonCommand.MOVE_POWER_CELLS, m_controlBoard.buttonBox.green);
    m_buttonMap.put(ButtonCommand.MOVE_POWER_CELLS_REVERSE, m_controlBoard.buttonBox.yellow);


    m_buttonMap.put(ButtonCommand.AUTO_AIM_TURRET, m_controlBoard.buttonBox.bigRed);
    

    m_buttonMap.put(ButtonCommand.BOOST_SPEED, m_controlBoard.extreme.joystickTopRight);
    m_buttonMap.put(ButtonCommand.CUT_SPEED, m_controlBoard.extreme.joystickBottomRight);












    m_buttonMap.put(ButtonCommand.SHIFT_DRIVETRAIN, m_controlBoard.xbox.rightBumper);
    m_buttonMap.put(ButtonCommand.EXTEND_INTAKE, m_controlBoard.xbox.aButton);
    m_buttonMap.put(ButtonCommand.RETRACT_INTAKE, m_controlBoard.xbox.yButton);

    m_buttonMap.put(ButtonCommand.INTAKE_EXTENDER_MOTION_MAGIC, m_controlBoard.extreme.joystickTopLeft);
    m_buttonMap.put(ButtonCommand.MANUAL_RUN_SHOOTER, m_controlBoard.extreme.trigger);
    m_buttonMap.put(ButtonCommand.STOP_MOTORS, m_controlBoard.extreme.sideButton);
    // m_buttonMap.put(ButtonCommand.TURRET_TESTING_MOTION_MAGIC, m_controlBoard.extreme.baseFrontLeft);
    

    m_buttonMap.put(ButtonCommand.AUTO_REFILL_QUEUE, m_controlBoard.extreme.baseBackLeft);
    m_buttonMap.put(ButtonCommand.AUTO_RUN_SHOOTER, m_controlBoard.extreme.baseMiddleRight);
    m_buttonMap.put(ButtonCommand.AUTO_SHOOT, m_controlBoard.extreme.baseMiddleLeft);
    m_buttonMap.put(ButtonCommand.AUTONOMOUS, m_controlBoard.extreme.baseFrontLeft);
    m_buttonMap.put(ButtonCommand.TOGGLE_COMPRESSOR, m_controlBoard.extreme.baseFrontRight);


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
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {









  // the actually important ones

    m_buttonMap.get(ButtonCommand.MANUAL_RUN_INTAKE).whileHeld(Commands.runIntake(m_intake, m_intakeSpeed));
    m_buttonMap.get(ButtonCommand.MANUAL_RUN_INTAKE_REVERSE).whileHeld(Commands.runIntake(m_intake, -m_intakeSpeed));


    m_buttonMap.get(ButtonCommand.MANUAL_RUN_MAGAZINE).whileHeld(Commands.runMagazine(m_magazine, m_magazineSpeed));
    m_buttonMap.get(ButtonCommand.MANUAL_RUN_MAGAZINE_REVERSE).whileHeld(Commands.runMagazine(m_magazine, -m_magazineSpeed));


    m_buttonMap.get(ButtonCommand.MANUAL_RUN_QUEUE).whileHeld(Commands.runQueue(m_queue, m_queueSpeed));
    m_buttonMap.get(ButtonCommand.MANUAL_RUN_QUEUE_REVERSE).whileHeld(Commands.runQueue(m_queue, -m_queueSpeed));


    // m_buttonMap.get(ButtonCommand.BOOST_SPEED).whenPressed(Commands.boostSpeed(m_shooter, 0.1));
    // m_buttonMap.get(ButtonCommand.CUT_SPEED).whenPressed(Commands.boostSpeed(m_shooter, -0.1));


    


  // move power cells
    m_buttonMap.get(ButtonCommand.MOVE_POWER_CELLS).whileHeld(Commands.runMagazine(m_magazine, m_magazineSpeed)
        .alongWith(Commands.runQueue(m_queue, m_queueSpeed)).alongWith(Commands.runIntake(m_intake, m_intakeSpeed)));
    m_buttonMap.get(ButtonCommand.MOVE_POWER_CELLS_REVERSE)
        .whileHeld(Commands.runIntake(m_intake, -m_intakeSpeed).alongWith(
            Commands.runMagazine(m_magazine, -m_magazineSpeed).alongWith(Commands.runQueue(m_queue, -m_queueSpeed))));



    m_buttonMap.get(ButtonCommand.SHOOT_BALL)
      .whileHeld(Commands.shootBallsAndAim(m_shooter, m_queue, m_queueSpeed,
        m_magazine, m_magazineSpeed, MagazinePowerCellCounter.getCount(), m_turret, m_limelight));


  //auto aim

    if (m_limelightEnabled) {
      m_buttonMap.get(ButtonCommand.AUTO_AIM_TURRET).toggleWhenPressed(Commands.findTarget(m_turret, m_limelight));
    }








    //everything else

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

    

    // Add auto refill qeuue
    
    m_buttonMap.get(ButtonCommand.MANUAL_RUN_SHOOTER).whileHeld(Commands.setShooterVelocity(m_shooter, m_shooter::getCalculatedVelocity));
    m_buttonMap.get(ButtonCommand.STOP_MOTORS).toggleWhenPressed(Commands.stopMotors(m_magazine, m_queue, m_shooter));

    m_buttonMap.get(ButtonCommand.AUTONOMOUS).whileHeld(Commands.autoCommand(m_shooter, m_queue, m_queueSpeed, m_turret, m_limelight, m_magazine, m_magazineSpeed, 3, m_drivetrain, 1.0));

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


    // m_buttonMap.get(ButtonCommand.TURRET_TESTING_MOTION_MAGIC).whileHeld(Commands.runTurretPosition(m_turret, 0.0));
    
    m_buttonMap.get(ButtonCommand.AUTO_RUN_SHOOTER)
        .toggleWhenPressed(Commands.setShooterVelocity(m_shooter, m_shooter::getCalculatedVelocity));
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
    if (extendButton) intakeExtended = extendButton;
    else if(retractButton) intakeExtended = !retractButton;
    SmartDashboard.putBoolean("intake extended", intakeExtended);
    return intakeExtended;
  }

  public boolean runExtenderMotionMagic() {
    boolean toggleButton = m_buttonMap.get(ButtonCommand.INTAKE_EXTENDER_MOTION_MAGIC).get();
    if (toggleButton && !extenderMotionMagicButtonState) {motionMagic = !motionMagic;}
    extenderMotionMagicButtonState = toggleButton;
    SmartDashboard.putBoolean("motion magic enabled", motionMagic);
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
    if (m_climber.getCurrentCommand() != null) {m_climberEntry.forceSetBoolean(true);} else {m_climberEntry.forceSetBoolean(false);}
    if (m_compressor.getCurrentCommand() != null) {m_compressorEntry.forceSetBoolean(true);} else {m_compressorEntry.forceSetBoolean(false);}
    if (m_drivetrain.getCurrentCommand() != null) {m_drivetrainEntry.forceSetBoolean(true);} else {m_drivetrainEntry.forceSetBoolean(false);}
    if (m_intake.getCurrentCommand() != null) {m_intakeEntry.forceSetBoolean(true);} else {m_intakeEntry.forceSetBoolean(false);}
    if (m_magazine.getCurrentCommand() != null) {m_magazineEntry.forceSetBoolean(true);} else {m_magazineEntry.forceSetBoolean(false);}
    if (m_queue.getCurrentCommand() != null) {m_queueEntry.forceSetBoolean(true);} else {m_queueEntry.forceSetBoolean(false);}
    if (m_shooter.getCurrentCommand() != null) {m_shooterEntry.forceSetBoolean(true);} else {m_shooterEntry.forceSetBoolean(false);}
    if (m_spinner.getCurrentCommand() != null) {m_spinnerEntry.forceSetBoolean(true);} else {m_spinnerEntry.forceSetBoolean(false);}
    if (m_turret.getCurrentCommand() != null) {m_turretEntry.forceSetBoolean(true);} else {m_turretEntry.forceSetBoolean(false);}
    // m_magazinePowerCellCounter.updateCount();
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
