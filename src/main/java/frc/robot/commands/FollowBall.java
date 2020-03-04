/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.CommandLogger;
import frc.robot.vision.PiTable;
import frc.robot.vision.Limelight.Pipeline;

public class FollowBall extends CommandLogger {
  private Drivetrain m_drivetrain;
  private PiTable m_table;
  private Pipeline m_pipeline;

  //constants to change the moving
  private final double k_move = 0.2;
  private final double k_turn = 0.03;

  //move and turn
  private double move;
  private double turn;

  public FollowBall(Drivetrain drivetrain, PiTable table, Pipeline pipeline) {
    m_table = table;
    m_pipeline = pipeline;
    m_drivetrain = drivetrain;
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_table.hasTarget()) {
      move = m_table.getDistance(m_table.getClosestTarget()) * k_move;
      turn = m_table.getXOffset(m_table.getClosestTarget()) * k_turn;
    }
    SmartDashboard.putNumber("move", move);
    SmartDashboard.putNumber("turn", turn);
    m_drivetrain.simpleArcadeDrive(move, turn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.simpleArcadeDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
