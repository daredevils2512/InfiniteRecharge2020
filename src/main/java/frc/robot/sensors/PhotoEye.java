package frc.robot.sensors;

public class PhotoEye extends DevilDigitalInput {
  
  public PhotoEye(int channel) {
    super(channel);
  }

  public boolean get() {
    return !super.get();
  }
}