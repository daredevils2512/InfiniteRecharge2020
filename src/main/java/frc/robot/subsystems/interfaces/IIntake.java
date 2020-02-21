package frc.robot.subsystems.interfaces;

import java.util.Map;

public interface IIntake {
  public void runIntake(double speed);

  public void setMotionMagicEnabled(boolean wantsEnabled);
  public void resetIntakeExtenderAngle();
  public boolean getExtended();
  public void setExtended(boolean wantsExtended);
  public void runExtender(double output);

  public Map<String, Object> getValues();
}
