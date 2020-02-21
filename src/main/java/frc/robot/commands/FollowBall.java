/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.vision.LimelightPipeline;

public class FollowBall extends CommandBase {
  private Drivetrain m_drivetrain;
  private LimelightPipeline m_pipeline;

  //constants to change the moving
  private final double m_maxMove = 0.2;
  private final double m_maxTurn = 0.03;
  private final double m_targetFill = 0.7;

  public FollowBall(Drivetrain drivetrain, LimelightPipeline pipeline) {
    m_drivetrain = drivetrain;
    m_pipeline = pipeline;
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.simpleArcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
