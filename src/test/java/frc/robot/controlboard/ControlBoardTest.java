package frc.robot.controlboard;

import static org.junit.Assert.assertEquals;
import org.junit.Test;

public class ControlBoardTest {

    @Test
    public void testButtoneyThing() {
       ControlBoard controlBoard = new ControlBoard("autoRefillQueue=extreme.joystickTopLeft \n runIntake=xbox.yButton \n runQueue=extreme.joystickTopRight");
       assertEquals(controlBoard.extreme.joystickTopLeft, controlBoard.getButton("autoRefillQueue"));
       assertEquals(controlBoard.xbox.yButton, controlBoard.getButton("runIntake"));
       assertEquals(controlBoard.extreme.joystickTopRight, controlBoard.getButton("runQueue"));
    }
}