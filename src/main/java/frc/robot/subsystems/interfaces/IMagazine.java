package frc.robot.subsystems.interfaces;

import java.util.logging.*;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.sensors.DummyDigitalInput;
import frc.robot.sensors.IDigitalInput;
import frc.robot.sensors.IDigitalInput;
import frc.robot.sensors.PhotoEye;

public interface IMagazine {
  public boolean getPowerCellDetectedFront();
  public boolean getPowerCellDetectedBack();
  public int getPowerCellCount();
  public void setBallsInMag(int set);
  public void resetBallCount();
  public void updatePowerCellCount();
  public boolean getDirectionReversed();

  public void setSpeed(double speed);
  public void feedBalls(int amount);
  
  public Map<String, Object> getValues();
}
