/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sensors.ColorSensor.ColorDetect;
import frc.robot.subsystems.Spinner;

public class PrecisionControl extends CommandBase {
  private final Spinner m_spinner;
  private ColorDetect m_targetColor;
  // TODO: get the color of the percision control dispaed in the part of the smart dashboard 
 // TODO: (ask game manual 4 help? ) 
  /**
   * Creates a new PrecisionControl.
   */
  public PrecisionControl(Spinner spinner, ColorDetect targetColor) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_spinner = spinner;
    addRequirements(m_spinner);
    m_targetColor = targetColor;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // has this constantly running as the program is being run. 
    m_spinner.run(0.25);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // this happens if the isFinished funcTion is called 
    m_spinner.run(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // retuns true if the currnet color is dthe same as the target color 
    return m_spinner.getCurrentColor() == m_targetColor;
  }
}
