/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;
import java.util.logging.Level;
import java.util.logging.Logger;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.spline.Spline;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.ControlVectorList;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.interfaces.IClimber;
import frc.robot.subsystems.interfaces.ICompressorManager;
import frc.robot.subsystems.interfaces.IDrivetrain;
import frc.robot.subsystems.interfaces.IIntake;
import frc.robot.subsystems.interfaces.IMagazine;
import frc.robot.subsystems.interfaces.IQueue;
import frc.robot.subsystems.interfaces.IShooter;
import frc.robot.subsystems.interfaces.ISpinner;
import frc.robot.subsystems.interfaces.ITurret;
import frc.robot.vision.PiTable;
import frc.robot.vision.Limelight;
import frc.robot.vision.LimelightLEDMode;
import frc.robot.RobotContainer;
import frc.robot.sensors.ColorSensor.ColorDetect;

/**
 * Definitions for all commands
 * 
 * <p>
 * The reasoning here is that whether a command should be inlined or not should
 * be up to the discretion of whoever is writing command based on their
 * complexity and specific implementation, but anyone using commands should be
 * able to access them all in the same manner, regardless of whether they are
 * inlined or not. This class is meant to make both requirements possible and
 * avoid overflowing the {@link RobotContainer} class with hard-to-read command
 * definitions.
 */
public final class Commands {
  private static Logger logger = Logger.getLogger(Commands.class.getName());

  private Commands() {
  }

  public static Command toggleClimberExtended(IClimber climber) {
    return new InstantCommand(() -> climber.toggleClimberExtended());
  }

  public static Command setClimberExtended(IClimber climber, boolean wantsExtended) {
    return new InstantCommand(() -> climber.extendClimbers(wantsExtended));
  }

  public static Command extendClimber(IClimber climber) {
    return new InstantCommand(() -> climber.extendClimbers(true));
  }

  public static Command retractClimber(IClimber climber) {
    return new InstantCommand(() -> climber.extendClimbers(false));
  }

  public static Command runClimber(IClimber climber, double leftSpeed, double rightSpeed) {
    return new InstantCommand(() -> climber.climb(leftSpeed, rightSpeed));
  }

  public static Command runDriveClimb(IDrivetrain drivetrain, double leftSpeed, double rightSpeed) {
    return new InstantCommand(() -> drivetrain.drivetrainClimb(leftSpeed, rightSpeed));
  }

  public static Command setLimelightLEDMode(Limelight limelight, LimelightLEDMode ledMode) {
    return new InstantCommand(() -> limelight.setLEDMode(ledMode));
  }

  /**
   * Sets the limelight LED mode to force OFF if not currently set to OFF,
   * otherwise revert to mode set by pipeline
   * 
   * @param limelight Limelight to set LED mode on
   * @return New {@link Command}
   */
  public static Command toggleLimelightLEDForceOff(Limelight limelight) {
    return new InstantCommand(() -> {
      limelight.setLEDMode(
          limelight.getLEDMode() == LimelightLEDMode.OFF ? LimelightLEDMode.PIPELINE : LimelightLEDMode.OFF);
    });
  }

  /**
   * Simple percent output arcade drive
   * 
   * @param drivetrain   Drivetrain to use
   * @param moveSupplier Forward speed supplier (-1 to +1)
   * @param turnSupplier Turn speed supplier (-1 to +1)
   * @return New {@link Command}
   */
  public static Command simpleArcadeDrive(IDrivetrain drivetrain, DoubleSupplier moveSupplier,
      DoubleSupplier turnSupplier) {
    return new RunCommand(() -> drivetrain.simpleArcadeDrive(moveSupplier.getAsDouble(), turnSupplier.getAsDouble()),
        drivetrain);
  }

  /**
   * Arcade drive with PID velocity control
   * 
   * @param drivetrain   Drivetrain to use
   * @param moveSupplier Forward speed supplier (-1 to +1)
   * @param turnSupplier Turn speed supplier (-1 to +1)
   * @return New {@link Command}
   */
  public static Command velocityArcadeDrive(IDrivetrain drivetrain, DoubleSupplier moveSupplier,
      DoubleSupplier turnSupplier) {
    DoubleSupplier velocitySupplier = () -> moveSupplier.getAsDouble() * drivetrain.getMaxSpeed();
    DoubleSupplier angularVelocitySupplier = () -> turnSupplier.getAsDouble() * drivetrain.getMaxAngularSpeed();
    return new RunCommand(
        () -> drivetrain.velocityArcadeDrive(velocitySupplier.getAsDouble(), angularVelocitySupplier.getAsDouble()),
        drivetrain);
  }

  public static Command accelerationLimitedSimpleArcadeDrive(IDrivetrain drivetrain, DoubleSupplier moveSupplier,
      DoubleSupplier turnSupplier, double maxMoveAcceleration, double maxTurnAcceleration) {
    return new AccelerationLimitedSimpleArcadeDrive(drivetrain, moveSupplier, turnSupplier, maxMoveAcceleration,
        maxTurnAcceleration);
  }

  public static Command accelerationLimitedVelocityArcadeDrive(IDrivetrain drivetrain, DoubleSupplier moveSupplier,
      DoubleSupplier turnSupplier, double maxMoveAcceleration, double maxTurnAcceleration) {
    return new AccelerationLimitedVelocityArcadeDrive(drivetrain, moveSupplier, turnSupplier, maxMoveAcceleration,
        maxTurnAcceleration);
  }

  public static Command driveStraight(IDrivetrain drivetrain, double distance) {
    logger.fine("running");
    return new DriveStraight(drivetrain, distance);
  }

  public static Command setDrivingInverted(IDrivetrain drivetrain, boolean wantsInverted) {
    logger.fine("inverted to" + wantsInverted);
    return new InstantCommand(() -> drivetrain.setDrivingInverted(wantsInverted), drivetrain);
  }

  public static Command toggleInvertedDriving(IDrivetrain drivetrain) {
    logger.fine("inverted to" + !drivetrain.getDrivingInverted());
    return new InstantCommand(() -> drivetrain.setDrivingInverted(!drivetrain.getDrivingInverted()), drivetrain);
  }

  public static Command drivetrainSetLowGear(IDrivetrain drivetrain, boolean wantsLowGear) {
    logger.fine("low gear = " + wantsLowGear);
    return new InstantCommand(() -> drivetrain.setLowGear(wantsLowGear), drivetrain);
  }

  public static Command drivetrainToggleLowGear(IDrivetrain drivetrain) {
    logger.fine("low gear = " + !drivetrain.getLowGear());
    return new InstantCommand(() -> drivetrain.setLowGear(!drivetrain.getLowGear()), drivetrain);
  }

  // probly temporary
  public static Command climberUp(IClimber climber, DoubleSupplier leftSpeed, DoubleSupplier rightSpeed) {
    logger.fine("left speed = " + leftSpeed + "right speed = " + rightSpeed);
    return new RunCommand(() -> climber.climb(leftSpeed.getAsDouble(), rightSpeed.getAsDouble()), climber);
  }

  /**
   * Extends and starts running the power cell intake
   * 
   * @return New {@link Command}
   */
  public static Command startIntaking(IIntake intake, IMagazine magazine) {
    logger.fine("running");
    return new InstantCommand(() -> intake.setExtended(true), intake)
        .andThen(new RunCommand(() -> magazine.setSpeed(1), magazine));
  }

  /**
   * Stops running and retracts the power cell intake
   * 
   * @return New {@link Command}
   */
  public static Command stopIntaking(IIntake intake, IMagazine magazine) {
    logger.fine("stopping");
    return new InstantCommand(() -> magazine.setSpeed(0), magazine)
        .andThen(new InstantCommand(() -> intake.setExtended(false), intake));
  }

  public static Command runIntake(IIntake intake, double speed) {
    logger.fine("running at" + speed);
    return new RunIntake(intake, speed);
  }

  public static Command runIntakeExtender_Temp(IIntake intake, DoubleSupplier speedSupplier) {
    return new RunCommand(() -> intake.runExtender(speedSupplier.getAsDouble()), intake);
  }

  public static Command setIntakeExtended(IIntake intake, boolean wantsExtended) {
    logger.fine("set to" + wantsExtended);
    return new InstantCommand(() -> intake.setExtended(wantsExtended));
  }

  public static Command toggleIntakeExtended(IIntake intake) {
    return new ToggleIntakeExtended(intake);
  }

  public static Command runMagazine(IMagazine magazine, double speed) {
    logger.fine("running at" + speed);
    return new RunMagazine(magazine, speed);
  }

  public static Command refillQueue(IMagazine magazine, double magazineSpeed,
      IntSupplier magazinePowerCellCountSupplier, BooleanSupplier queueHasPowerCellSupplier) {
    return new RefillQueue(magazine, magazineSpeed, magazinePowerCellCountSupplier, queueHasPowerCellSupplier);
  }

  public static Command intakeCommand(IIntake intake, Supplier<Double> intakeAxis, double intakeSpeed,
      IMagazine magazine, double magazineSpeed, Supplier<Double> extenderSpeed, double extenderMaxSpeed,
      Supplier<Boolean> extended, Supplier<Boolean> shouldExtend) {
    return new IntakeCommand(intake, intakeAxis, intakeSpeed, magazine, magazineSpeed, extenderSpeed, extenderMaxSpeed,
        extended, shouldExtend);
  }

  public static Command runQueue(IQueue queue, double speed) {
    logger.fine("running at" + speed);
    return new ManualRunQueue(queue, speed);
  }

  public static Command feedShooter(IQueue queue, DoubleSupplier queueSpeedSupplier) {
    return new FeedShooter(queue, queueSpeedSupplier);
  }

  public static Command autoFeedShooter(IQueue queue, double queueSpeed, IShooter shooter) {
    return new AutoFeedShooter(queue, queueSpeed, shooter);
  }

  public static Command moveTurret(ITurret turret, DoubleSupplier speedSupplier) {
    logger.fine("running at" + speedSupplier);
    return new RunCommand(() -> turret.setSpeed(speedSupplier.getAsDouble()), turret);
  }

  public static Command runTurretPosition(ITurret turret, double position) {
    logger.fine("ran turret postition" + position);
    return new RunCommand(() -> turret.runPosition(position), turret);
  }

  public static Command findTarget(ITurret turret, Limelight limelight) {
    logger.info("finding traget");
    return new FindTarget(turret, limelight);
  }

  public static Command stopMotors(IMagazine magazine, IQueue queue, IShooter shooter) {
    logger.info("stopping motors");
    return Commands.runMagazine(magazine, 0.0).alongWith(Commands.runQueue(queue, 0.0))
        .alongWith(Commands.runShooter(shooter, () -> 0.0));
  }

  /**
   * Set shooter percent output
   * 
   * @param shooter
   * @return New {@link Command}
   */
  public static Command runShooter(IShooter shooter, DoubleSupplier speedSupplier) {
    return new RunCommand(() -> shooter.setPercentOutput(speedSupplier.getAsDouble()), shooter);
    // return new RunCommand(() ->
    // shooter.setPercentOutput(speedSupplier.getAsDouble()), shooter);
  }

  public static Command setShooterVelocity(IShooter shooter, Supplier<Double> speed) {
    return new RunShooterPID(shooter, speed);
  }

  public static Command setShooterAngle(IShooter shooter, DoubleSupplier angleSupplier) {
    return new RunCommand(() -> shooter.setTargetAngle(angleSupplier.getAsDouble()), shooter);
  }

  public static Command runHood(IShooter shooter, DoubleSupplier supplier) {
    return new RunCommand(() -> shooter.setHoodSpeed(supplier.getAsDouble()))
        .andThen(new RunCommand(() -> shooter.setHoodSpeed(0.0)));
  }

  public static Command stopShooter(IShooter shooter) {
    return new InstantCommand(shooter::stop, shooter);
  }

  public static Command setSpinnerExtended(ISpinner spinner, boolean wantsExtended) {
    return new InstantCommand(() -> spinner.setExtended(wantsExtended), spinner);
  }

  public static Command rotationControl(ISpinner spinner, double rotations) {
    return new RotationControl(spinner, rotations);
  }

  public static Command precisionControl(ISpinner spinner, ColorDetect targetColor) {
    return new PrecisionControl(spinner, targetColor);
  }
  public static Command extendClimbers(IClimber climber){
    return new InstantCommand(()-> climber.extendClimbers(true));
  }
  public static Command retractClimber(IClimber climber){
    return new InstantCommand(()-> climber.extendClimbers(false));
  }
  public static Command toggleCompressor(ICompressorManager compressor) {
    return new InstantCommand(() -> compressor.toggleCompressor());
  }

  /**
   * calls shootBalls and FindTarget and ends when shootBalls has shot however many balls balls
   * @param shooter all theses
   * @param queue are
   * @param queueSpeed <h1> INJECTED SUBSYSTEMD
   * @param magazine
   * @param magazineSpeed
   * @param balls
   * @return a DeadlineCommandGroup that ends and interrupts everything when shootBalls ends
   */
  public static Command shootBallsAndAim(IShooter shooter, IQueue queue, double queueSpeed, IMagazine magazine,
      double magazineSpeed, int balls, ITurret turret, Limelight limelight) {
    return new ShootBalls(shooter, queue, queueSpeed, magazine, magazineSpeed, balls)
        .alongWith(Commands.findTarget(turret, limelight));
  }

  public static Command shootBalls(IShooter shooter, IQueue queue, double queueSpeed, IMagazine magazine,
  double magazineSpeed, int balls) {
    return new ShootBalls(shooter, queue, queueSpeed, magazine, magazineSpeed, balls);
  }

  //made a separate class for this
  // public static Command shootBalls(IShooter shooter, IQueue queue, double queueSpeed, ITurret turret,
  //     Limelight limelight, IMagazine magazine, double magazineSpeed, int balls) {
  //   double startingCount = MagazinePowerCellCounter.getCount();
  //   logger.log(Level.INFO, "Commands::shootBalls(QueueSPeed: " + queueSpeed + ", MagazineSpeed: " + magazineSpeed
  //       + ", balls: " + balls + " starting count: " + startingCount);

  //   return new RunCommand(() -> shooter.setTargetVelocity(shooter.getCalculatedVelocity()), shooter)
  //       .alongWith(Commands.runQueue(queue, queueSpeed), Commands.runMagazine(magazine, magazineSpeed))
  //       .alongWith(Commands.findTarget(turret, limelight))
  //       .withInterrupt(() -> MagazinePowerCellCounter.getCount() <= startingCount - balls)
  //       .andThen(Commands.setShooterVelocity(shooter,() -> 0.0));
  // }

  public static Command autoCommand(IShooter shooter, IQueue queue, double queueSpeed, ITurret turret,
      Limelight limelight, IMagazine magazine, double magazineSpeed, int balls, IDrivetrain drivetrain,
      double distance) {

    logger.log(Level.INFO, "ran auto command");
    return Commands.shootBallsAndAim(shooter, queue, queueSpeed, magazine, magazineSpeed, balls, turret, limelight).withTimeout(5)
        .andThen(Commands.driveDistance(drivetrain, distance, 0.3));
    // return Commands.driveDistance(drivetrain, distance, 0.3);
  }

  public static Command boostSpeed(IShooter shooter, double boost) {
    logger.info("loaded boost speed command for boost " + boost);
    return new InstantCommand(() -> shooter.boostSpeed(boost));
  }

  /**
   * drives a distance without using pid
   * @param drivetrain injected drivetrain
   * @param distance distance
   * @return command
   */
  public static Command driveDistance(IDrivetrain drivetrain, double distance, double speed) {
    // double startPos = (drivetrain.getRightDistance() + drivetrain.getLeftDistance()) / 2;
    // logger.log(Level.INFO, "drive distance with start pos " + startPos + " and distance " + distance + "at speed " + speed);
    // return Commands.simpleArcadeDrive(drivetrain, () -> speed * Math.signum(distance), () -> 0.0)
    //     .withInterrupt(() -> Math.abs(((drivetrain.getLeftDistance() + drivetrain.getRightDistance()) / 2) - startPos) >= Math.abs(distance));
    return new DriveDistance(drivetrain, distance, speed);
  }

  public static Command findBall(IDrivetrain drivetrain, PiTable table) {
    Translation2d translation = table.getClosestBallPose().getTranslation();
    TrajectoryConfig config = new TrajectoryConfig(3.0, 3.0);
    ControlVectorList vectors = new ControlVectorList();
    Spline.ControlVector vector = new Spline.ControlVector(new double[] { translation.getX() },
        new double[] { translation.getY() });
    vectors.add(vector);
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(vectors, config);

    return new RamseteCommand(trajectory, drivetrain::getPose, new RamseteController(), drivetrain.getFeedForward(),
        drivetrain.getKinematics(), drivetrain::getWheelSpeeds, drivetrain.getLeftController(),
        drivetrain.getRightController(), drivetrain::voltageTank, drivetrain)
            .andThen(() -> drivetrain.simpleArcadeDrive(0, 0));
  }

  public static Command followPath(IDrivetrain drivetrain, String file) {
    Trajectory trajectory;
    try {
      Path path = Filesystem.getDeployDirectory().toPath().resolve("paths/" + file);
      trajectory = TrajectoryUtil.fromPathweaverJson(path);
    } catch (IOException e) {
      trajectory = null;
      e.printStackTrace();
    }
    return new RamseteCommand(trajectory, drivetrain::getPose, new RamseteController(), drivetrain.getFeedForward(),
        drivetrain.getKinematics(), drivetrain::getWheelSpeeds, drivetrain.getLeftController(),
        drivetrain.getRightController(), drivetrain::voltageTank, drivetrain)
            .andThen(() -> drivetrain.simpleArcadeDrive(0, 0));
  }
}