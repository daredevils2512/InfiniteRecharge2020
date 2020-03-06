package frc.robot.subsystems.interfaces;

public interface IClimber extends IPropertySubsystem {
  public void climb(double leftSpeed, double rightSpeed);
  public void extendClimbers(boolean wantsExtended);
  public void toggleClimberExtended();
}