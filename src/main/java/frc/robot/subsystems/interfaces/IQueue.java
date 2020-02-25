package frc.robot.subsystems.interfaces;

public interface IQueue extends IPropertySubsystem {
  public void run(double speed);
  public boolean getDirectionReversed();
  public boolean hasPowerCell();
}
