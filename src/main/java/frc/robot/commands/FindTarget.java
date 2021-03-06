/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.interfaces.ITurret;
import frc.robot.utils.CommandLogger;
import frc.robot.vision.Limelight;
import frc.robot.vision.LimelightLEDMode;
import frc.robot.vision.Limelight.Pipeline;

public class FindTarget extends CommandLogger {
  private ITurret m_turret;
  private Limelight m_limelight;
  private NetworkTable m_networkTable;


  /**
   * Creates a new FindTarget.
   */
  public FindTarget(ITurret turret, Limelight limelight) {
    m_turret = turret;
    m_limelight = limelight;
    m_networkTable = NetworkTableInstance.getDefault().getTable("hexagon position");
    addRequirements(m_turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_limelight.setLEDMode(LimelightLEDMode.ON);
    m_limelight.setPipeline(Pipeline.Hexagon);
    m_logger.fine("finding target at: " + m_networkTable.getEntry("turret relative position").getDouble(m_turret.getAngle()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double target = m_networkTable.getEntry("turret relative position").getDouble(m_turret.getAngle());
    // SmartDashboard.putNumber("target position", m_networkTable.getEntry("turret relative position").getDouble(m_turret.getAngle()));
    m_turret.runPosition(target);
    // m_turret.runPosition(m_turret.getAngle() + m_limelight.tx());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_logger.info("target findiing done. was interrupted: " + interrupted);
    m_turret.setSpeed(0.0);
    m_limelight.setLEDMode(LimelightLEDMode.OFF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return (Math.abs(m_networkTable.getEntry("turret relative position").getDouble(m_turret.getAngle()) - m_turret.getAngle()) <= m_tolerance);
    return false;
  }
}
