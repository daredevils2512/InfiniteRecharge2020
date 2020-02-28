package frc.robot;

import static org.junit.Assert.assertNotNull;

import org.junit.Test;

public class RobotContainerTest {

    /**
     * will make a bunch of talon errors and stuff thats fine but the test should pass
     */
    @Test
    public void testRobotContainer() {
        RobotContainer robotConatiner = new RobotContainer();
        assertNotNull(robotConatiner);
    }
}