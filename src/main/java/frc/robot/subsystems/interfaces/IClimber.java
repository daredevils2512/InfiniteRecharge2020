package frc.robot.subsystems.interfaces;

public interface IClimber extends IPropertySubsystem {
  public void extendClimbers(double leftSpeed, double rightSpeed);
  public void raiseClimbers(boolean wantsExtended);
  public void toggleClimberExtended();
  public void extendLeftClimber(double speed);
  public void extendRightClimber(double speed);
  public void resetEncoders();
  public int getLeftEncoder();
  public int getRightEncoder();
}