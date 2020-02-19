package frc.robot.utils;

import static org.junit.Assert.assertEquals;
import org.junit.Test;

public class DareMathUtilTest {

    @Test
    public void testMapRange() {
        assertEquals(0.5, DareMathUtil.mapRange(1.0, 0.0, 2.0, 0.0, 1.0), 1e-15);
        assertEquals(1.0, DareMathUtil.mapRange(2.0, 0.0, 4.0, 0.0, 2.0), 1e-15);
    }
}