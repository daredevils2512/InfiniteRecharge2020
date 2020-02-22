package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

public class PIDWrapper {
  private double Kp;
  private double Ki;
  private double Kd;

  public PIDWrapper(double p, double i, double d) {
    Kp = p;
    Ki = i;
    Kd = d;
  }

  public void configPID(BaseMotorController motorController, int PIDslot) {
    motorController.config_kP(PIDslot, Kp);
    motorController.config_kI(PIDslot, Ki);
    motorController.config_kD(PIDslot, Kd);
  }

  public void setPID(double p, double i, double d) {
    Kp = p;
    Ki = i;
    Kd = d;
  }

  public double getP() {
    return Kp;
  }

  public double getI() {
    return Ki;
  }

  public double getD() {
    return Ki;
  }
}