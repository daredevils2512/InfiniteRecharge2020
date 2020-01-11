/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.vision.Limelight.Pipeline;

public class FollowBall extends CommandBase {
  private Drivetrain m_drivetrain;
  private Pipeline m_pipeline;

  //constants to change the moving
  private final double k_move = 0.2;
  private final double k_turn = 0.03;
  private final double targetArea = 70;

  //move and turn
  private double move;
  private double turn;

  public FollowBall(Drivetrain drivetrain, Pipeline pipeline) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_pipeline = pipeline;
    m_drivetrain = drivetrain;
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.m_limelight.setPipeline(m_pipeline);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Robot.m_limelight.hasTarget()) {
      move = -Math.min(((targetArea - Robot.m_limelight.ta()) * k_move), 0.8);
      turn = -(Robot.m_limelight.tx() * k_turn) <= 0.7 && Robot.m_limelight.tx() * k_turn >= -0.7 ? 
        Robot.m_limelight.tx() * k_turn : 0.5 * Math.signum(Robot.m_limelight.tx());
    } else {
      turn = -0.5 * Math.signum(Robot.m_limelight.getLastPosition());
      move = 0.1;
    }
    SmartDashboard.putNumber("move", move);
    SmartDashboard.putNumber("turn", turn);
    m_drivetrain.arcadeDrive(move, turn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.arcadeDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
