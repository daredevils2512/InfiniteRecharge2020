package frc.robot.controlboard;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.function.Supplier;

import org.junit.Test;

public class ControlBoardTest {

    @Test
    public void testButtoneyThing() {
        ControlBoard controlBoard = new ControlBoard("autoRefillQueue=extreme.joystickTopLeft \n runIntake=xbox.yButton");
        assertEquals(controlBoard.extreme.joystickTopLeft, controlBoard.getButton("autoRefillQueue"));
        assertEquals(controlBoard.xbox.yButton, controlBoard.getButton("runIntake"));
    }

    @Test
    public void testJoystickThing() {
        ControlBoard controlBoard = new ControlBoard("testAxis=extreme.getStickY");
        Supplier<Double> supplier = () -> controlBoard.extreme.getStickY();
        System.out.println(controlBoard.getAxis("testAxis").toString());
        System.out.println(supplier.toString());
        assertEquals(supplier.get(), controlBoard.getAxis("testAxis").get());
    }
}