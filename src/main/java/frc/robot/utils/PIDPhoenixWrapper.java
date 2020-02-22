package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

public class PIDPhoenixWrapper extends PIDWrapper {
  private int k_PIDslot;

  public PIDPhoenixWrapper(double p, double i, double d) {
    super(p, i, d);
    k_PIDslot = 0;
  }

  public PIDPhoenixWrapper(double p, double i, double d, int pidSlot) {
    super(p, i, d);
    k_PIDslot = pidSlot;
  }

  public void setPID(double p, double i, double d, int pidSlot) {
    k_PIDslot = pidSlot;
    setPID(p, i, d);
  }

  public void configPID(BaseMotorController motorController) {
    configPID(motorController, k_PIDslot);
  }
}