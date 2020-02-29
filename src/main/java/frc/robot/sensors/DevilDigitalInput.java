package frc.robot.sensors;

import edu.wpi.first.wpilibj.DigitalInput;

public abstract class DevilDigitalInput extends DigitalInput implements IDigitalInput {
 
 //   protected  final DigitalInput m_digitalInput;

    public DevilDigitalInput(int port){
      super(port);
        //this.m_digitalInput = new DigitalInput(port);
    }

    //public boolean get()

}
