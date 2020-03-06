/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.interfaces.IDrivetrain;
import frc.robot.utils.CommandLogger;
import frc.robot.utils.DareMathUtil;

public class DriveDistance extends CommandLogger {
  
  private IDrivetrain m_drivetrain;
  private double m_distance, m_speed, m_targetPosition = 0.0;


  /**
   * Creates a new DriveDistance.
   */
  public DriveDistance(IDrivetrain drivetrain, double distance, double speed) {
    m_drivetrain = drivetrain;

    m_distance = distance;
    m_speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
    m_logger.info("initialised drive distance for " + distance + " at " + speed);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_targetPosition = m_drivetrain.getAverageDistance() + m_distance;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = m_speed * Math.signum(m_distance);
    m_drivetrain.simpleArcadeDrive(speed, 0.0);
    m_logger.fine("drove at " + speed);
  }  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.simpleArcadeDrive(0.0, 0.0);
    m_logger.info("drive distance finished with interrupted: " + interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return DareMathUtil.isWithinXOf(m_drivetrain.getAverageDistance(), m_targetPosition, 0.1);
  }
}
