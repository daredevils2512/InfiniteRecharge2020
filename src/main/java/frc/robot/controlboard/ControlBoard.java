/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.controlboard;

public class ControlBoard {
  private final int xboxPort = 1;
  private final int extremePort = 2;

  public final Xbox xbox = new Xbox(xboxPort);
  public final Extreme extreme = new Extreme(extremePort);
}
