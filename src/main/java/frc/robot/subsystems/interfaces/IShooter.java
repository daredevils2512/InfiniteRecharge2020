package frc.robot.subsystems.interfaces;

import java.util.HashMap;
import java.util.Map;
import java.util.Properties;
import java.util.logging.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.PropertyFiles;

public interface IShooter {
  public void resetHoodAngle(double angle);
  public void setPercentOutput(double speed);
  public void setTargetVelocity(double velocity);
  public void stop();
  public void setTargetAngle(double angle);
  public double getVelocity();
  public double getAngle();
  public Map<String, Object> getValues();
}
