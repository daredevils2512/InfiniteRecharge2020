package frc.robot.subsystems.dummy;

import frc.robot.subsystems.interfaces.ICompressorManager;

public class DummyCompressor implements ICompressorManager {
  @Override
  public void setClosedLoopControl(boolean wantsClosedLoopControl) {
    
  }

  @Override
  public boolean getClosedLoopControl() {
    return false;
  }

  @Override
  public void toggleCompressor() {
  }
  
}