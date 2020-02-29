package frc.robot.subsystems.interfaces;

public interface IMagazine extends IPropertySubsystem {
  void onPowerCellIn(Runnable runnable);
  void onPowerCellOut(Runnable runnable);
  boolean getPowerCellDetected();
  boolean getDirectionReversed();
  void setSpeed(double speed);
  void feedBalls(int amount);
}
