package frc.robot.controlboard;

import static org.junit.Assert.assertEquals;
import org.junit.Test;

public class ControlBoardTest {

    @Test
    public void testButtoneyThing() {
        ControlBoard controlBoard = new ControlBoard("autoRefillQueue=extreme.joystickTopLeft");
        assertEquals(controlBoard.extreme.joystickTopLeft, controlBoard.getButton("autoRefillQueue"));
    }
}