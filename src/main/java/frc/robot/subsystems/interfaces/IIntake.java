package frc.robot.subsystems.interfaces;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface IIntake extends Subsystem, IPropertySubsystem {
  public void runIntake(double speed);

  public void setMotionMagicEnabled(boolean wantsEnabled);
  public boolean getMotionMagicEnabled();
  public void toggleMotionMagicEnabled();
  public void resetIntakeExtenderAngle();
  public boolean getExtended();
  public void setExtended(boolean wantsExtended);
  public void toggleExtended();
  public void runExtender(double output);

  public Map<String, Object> getValues();
}
