/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.logging.Level;
import java.util.logging.Logger;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.interfaces.ITurret;

public class FindTarget extends CommandBase {
  private ITurret m_turret;
  private NetworkTable m_networkTable;
  private Logger turretLogger;


  /**
   * Creates a new FindTarget.
   */
  public FindTarget(ITurret turret) {
    m_turret = turret;
    turretLogger =  Logger.getLogger(m_turret.getClass().getName());
    m_networkTable = NetworkTableInstance.getDefault().getTable("hexagon position");
    addRequirements(m_turret);
    turretLogger.log(Level.INFO, "constructed turret tracking");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turretLogger.log(Level.INFO, "initializesed turret tracking");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double target = m_networkTable.getEntry("turret relative position").getDouble(m_turret.getAngle());
    // SmartDashboard.putNumber("target position", m_networkTable.getEntry("turret relative position").getDouble(m_turret.getAngle()));
    m_turret.runPosition(target);
    turretLogger.log(Level.INFO, "finding target at: " + target);
    // m_turret.runPosition(m_turret.getAngle() + m_limelight.tx());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turretLogger.log(Level.INFO, "target findiing done. was interrupted: " + interrupted);
    m_turret.setSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return (Math.abs(m_networkTable.getEntry("turret relative position").getDouble(m_turret.getAngle()) - m_turret.getAngle()) <= m_tolerance);
    return false;
  }
}
