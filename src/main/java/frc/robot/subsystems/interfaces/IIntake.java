package frc.robot.subsystems.interfaces;

import java.util.Map;

public interface IIntake extends IPropertySubsystem {
  public void runIntake(double speed);

  public void setMotionMagicEnabled(boolean wantsEnabled);
  public boolean getMotionMagicEnabled();
  public void toggleMotionMagicEnabled();
  public void resetIntakeExtenderAngle();
  public boolean getExtended();
  public void setExtended(boolean wantsExtended);
  public void toggleExtended();
  public void runExtender(double output);
  public void extend();
  public void retract();
  public boolean isMotionMagicFinished();

  public Map<String, Object> getValues();
}
