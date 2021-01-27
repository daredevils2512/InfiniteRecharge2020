package frc.robot.subsystems.interfaces;

public interface ICompressorManager extends ILogging {
  public void setClosedLoopControl(boolean wantsClosedLoopControl);
  public boolean getClosedLoopControl();
  public void toggleCompressor();
}