package frc.robot.subsystems.interfaces;

import java.util.Map;

public interface IQueue {
  public void run(double speed);
  public void run(double speed, boolean wantsClosed);
  public boolean getClosed();
  public void setClosed(boolean wantsClosed);
  public boolean hasPowerCell();
  public Map<String, Object> getValues();
}
