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

public class SpinnerCounter extends CommandBase {
  private final Spinner m_spinner;
  private ColorDetect pastColor;
  private int segmentCounter;

  /**
   * Creates a new SpinnerCounter.
   */
  public SpinnerCounter(Spinner spinner) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_spinner = spinner;
    addRequirements(m_spinner);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // get current color seen by sensor
    ColorDetect currentColor = m_spinner.getCurrentColor();
    // get past color and compare them.
    if (pastColor == currentColor) {
      // TODO: if they are the same then you know you need to move more.
    } else {
      //  if they are different then you have moved 1/8 or one segment.
      // keep track of how many 8ths you have moved.
      segmentCounter++;
    }

    // reset pastColor to current color
    pastColor = currentColor;

    // TODO: handle uknown variable 

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
