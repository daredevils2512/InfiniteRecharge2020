package frc.robot.subsystems.dummy;

import java.util.Map;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.interfaces.IClimber;
import frc.robot.subsystems.interfaces.IPropertySubsystem;

public class DummyClimber implements IClimber, IPropertySubsystem {

  @Override
  public void climb(double leftSpeed, double rightSpeed) {
  }

  @Override
  public void climbLeft(Drivetrain drivetrain, double speed) {
  }

  @Override
  public void climbRight(Drivetrain drivetrain, double speed) {
  }

  @Override
  public Map<String, Object> getValues() {
    return null;
  }

  @Override
  public void saveProperties() {
  }
}