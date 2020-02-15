/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;

/**
 * Add your docs here.
 */
public class CompressorManager {
  private Compressor compressor = new Compressor();
  private boolean m_compressorEnabled; 
  
  public void toggleCompressor(){
    m_compressorEnabled = !m_compressorEnabled;
    compressor.setClosedLoopControl(m_compressorEnabled);
  } 
}
