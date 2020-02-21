package frc.robot.subsystems;

import org.junit.Test;

import frc.robot.subsystems.dummy.DummyDrivetrain;
import frc.robot.subsystems.interfaces.IDrivetrain;

public class DrivetrainTest {
  @Test
  public void TestDrivetrain() {
    IDrivetrain drivetrain = new DummyDrivetrain();
  }
}
