package frc.robot.sensors;

import edu.wpi.first.wpilibj.DigitalInput;

public abstract class DevilDigitalInput implements IDigitalInput {
 
    protected  final DigitalInput m_digitalInput;

    public DevilDigitalInput(int port){
        this.m_digitalInput = new DigitalInput(port);
    }

    public boolean get() {
      return this.m_digitalInput.get();
    }
    
}
