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

public class RotationControl extends CommandBase {
  private final Spinner m_spinner;
  private final double m_rotations;
  private ColorDetect pastColor;
  private int segmentCounter;

  /**
   * Creates a new RotationControl.
   */
  public RotationControl(Spinner spinner, double rotations) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_spinner = spinner;
    m_rotations = rotations;
    addRequirements(m_spinner);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_spinner.run(1.0);
    // get current color seen by sensor
    ColorDetect currentColor = m_spinner.getCurrentColor();
    // if the current color is not the past color and the detected color is not unknown. 
    if (currentColor != pastColor && currentColor != ColorDetect.Unknown) {
      // the segment counter just counts the amount of color CHANGES it detects.
      segmentCounter++;
      // prints out the amount of segments moved in the DRIVER STATION CONSOLE not smart dashboard. 
      System.out.println("Moved " + segmentCounter + "segments");
      // resets current color to past color
      pastColor = currentColor;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // retuns true if value of the segment counter is greater than or equal to 3 * 8, or 24. 
    return segmentCounter >= m_rotations * 8;
  }
}
