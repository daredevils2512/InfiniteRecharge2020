package frc.robot.subsystems;

import static org.junit.Assert.assertEquals;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import org.junit.Test;

public class DrivetrainTest {

    @Test
    public void testMotorReflection() {
        assertEquals(new WPI_TalonFX(0), Drivetrain.getMotor("WPI_TalonFX", 0));
    }
}