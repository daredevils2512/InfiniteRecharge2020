package frc.robot.subsystems.dummy;

import java.util.Map;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.interfaces.IClimber;

public class DummyClimber implements IClimber {

  @Override
  public void climb(double leftSpeed, double rightSpeed) {
  }

  @Override
  public void climberMoveHorizontal(double speed) {
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
}