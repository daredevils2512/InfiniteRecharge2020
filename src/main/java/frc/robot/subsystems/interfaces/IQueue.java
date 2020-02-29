package frc.robot.subsystems.interfaces;

public interface IQueue extends IPropertySubsystem {
  void onPowerCellInMagazine(Runnable runnable);
  void onPowerCellInShooter(Runnable runnable);
  void onPowerCellOutMagazine(Runnable runnable);
  void onPowerCellOutShooter(Runnable runnable);
  void run(double speed);
  boolean getDirectionReversed();
  boolean hasPowerCell();
}
