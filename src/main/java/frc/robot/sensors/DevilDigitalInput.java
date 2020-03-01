package frc.robot.sensors;

import edu.wpi.first.wpilibj.DigitalInput;

public abstract class DevilDigitalInput implements IDigitalInput {
 
    protected DigitalInput m_digitalInput;

    // caleld by dummy's only.
    protected DevilDigitalInput(){

    }

    public DevilDigitalInput(int port){
      this.m_digitalInput = new DigitalInput(port);
    }

    public DigitalInput getDigitalInput(){
      return this.m_digitalInput;
    }

    public boolean get(){
      return this.m_digitalInput.get();
    }
}
