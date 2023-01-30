package frc.robot.sensors;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HallEffectSensor extends SubsystemBase{
    
    private DigitalInput limitSwitch;
    
    public HallEffectSensor(int channel){
        this.limitSwitch = new DigitalInput(channel);
    }

    public boolean get() {
        return this.limitSwitch.get();
    }
}
