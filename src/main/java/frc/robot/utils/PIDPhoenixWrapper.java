package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

public class PIDPhoenixWrapper extends PIDWrapper {
  private final int k_PIDslot = 0;

  public PIDPhoenixWrapper(double p, double i, double d) {
    super(p, i, d);
  }

  public void configPID(BaseMotorController motorController) {
    configPID(motorController, k_PIDslot);
  }
}