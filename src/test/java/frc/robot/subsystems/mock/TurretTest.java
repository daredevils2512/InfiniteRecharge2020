package frc.robot.subsystems.mock;

import static org.junit.Assert.assertEquals;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.junit.Test;


public class TurretTest {

    @Test
    public void testTurretInit() {
        //setup
        WPI_TalonSRX motor = mock(WPI_TalonSRX.class);
        MockTurret turret = new MockTurret(motor, 180, 1, 4096, 22 / 129);

        //test
        assertEquals(0, turret.getAngle(), 1e-15);
        verify(motor).getSelectedSensorPosition();
        verify(motor).setSelectedSensorPosition(0);
    }

    @Test
    public void testTurretMath() {
        WPI_TalonSRX motor = mock(WPI_TalonSRX.class);
        MockTurret turret = new MockTurret(motor, 180, 1, 4096, 22 / 129);

        assertEquals(0, turret.toDegrees(0), 1e-15);
    }
}