package frc.robot.utils;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;
import org.junit.Test;

public class DareMathUtilTest {

    @Test
    public void testMapRange() {
        assertEquals(0.5, DareMathUtil.mapRange(1.0, 0.0, 2.0, 0.0, 1.0), 1e-15);
        assertEquals(1.0, DareMathUtil.mapRange(2.0, 0.0, 4.0, 0.0, 2.0), 1e-15);
        assertNotEquals(0.0, DareMathUtil.mapRange(4.0, 3.0, 5.0, 0.5, 0.7), 1e-15);
    }

    @Test
    public void testWrap() {
        assertEquals(15, DareMathUtil.wrap(15, 10, 20), 1e-15);
        assertEquals(4, DareMathUtil.wrap(9, 0, 5), 1e-15);
        assertNotEquals(5.12536, DareMathUtil.wrap(115, 3.2367, 5.12536), 1e-15);
    }
}