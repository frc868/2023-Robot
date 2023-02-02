package frc.robot.subsystems;

import com.techhounds.houndutil.houndlog.LogGroup;
import com.techhounds.houndutil.houndlog.LogProfileBuilder;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.loggers.DeviceLogger;
import com.techhounds.houndutil.houndlog.loggers.Logger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * The manipulator class
 * 
 * @author gc
 */
public class Manipulator extends SubsystemBase {
    private DoubleSolenoid wrist = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.Manipulator.Solenoids.Wrist.FORWARD, Constants.Manipulator.Solenoids.Wrist.REVERSE);
    private DoubleSolenoid pincer = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.Manipulator.Solenoids.Pincer.FORWARD, Constants.Manipulator.Solenoids.Pincer.REVERSE);
    private PneumaticHub pneumaticHub = new PneumaticHub();
    private DigitalInput poleDetector = new DigitalInput(Constants.Manipulator.POLE_DETECTOR);

    public Manipulator() {
        pneumaticHub.enableCompressorDigital();
        
        LoggingManager.getInstance().addGroup("Manipulator", new LogGroup(
            new Logger[] {
                new DeviceLogger<DoubleSolenoid>(wrist, "Wrist Solenoid", LogProfileBuilder.buildDoubleSolenoidLogItems(wrist)),
                new DeviceLogger<DoubleSolenoid>(pincer, "Pincer", LogProfileBuilder.buildDoubleSolenoidLogItems(pincer))
            }
        ));
    }

    public void setWristUp() {
        wrist.set(Value.kForward); // untested
    }

    public void setWristDown() {
        wrist.set(Value.kReverse); // untested
    }

    public void setPincerUp() {
        pincer.set(Value.kForward); // untested
    }

    public void setPincerDown() {
        pincer.set(Value.kReverse); // untested
    }

    public boolean poleDetected() {
        return poleDetector.get(); // untested
    }
}
