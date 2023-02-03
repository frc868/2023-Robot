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
 * The manipulator class, containing the wrist, pincer, and pole detector
 * 
 * @author gc
 */
public class Manipulator extends SubsystemBase {
    /** The wrist of the manipulator */
    private DoubleSolenoid wrist = new DoubleSolenoid(PneumaticsModuleType.REVPH,
        Constants.Manipulator.Solenoids.Wrist.FORWARD,
        Constants.Manipulator.Solenoids.Wrist.REVERSE);

    /** Pincer to grab game pieces with */
    private DoubleSolenoid pincer = new DoubleSolenoid(PneumaticsModuleType.REVPH,
        Constants.Manipulator.Solenoids.Pincer.FORWARD,
        Constants.Manipulator.Solenoids.Pincer.REVERSE);

    /** Will become obsolete after vendor dep update */
    private PneumaticHub pneumaticHub = new PneumaticHub();

    /** Beam break sensor used to detect pole to put cones on */
    private DigitalInput poleDetector = new DigitalInput(Constants.Manipulator.POLE_DETECTOR);

    public Manipulator() {
        pneumaticHub.enableCompressorDigital();
        
        LoggingManager.getInstance().addGroup("Manipulator", new LogGroup(
            new Logger[] {
                new DeviceLogger<DoubleSolenoid>(wrist, "Wrist Solenoid",
                    LogProfileBuilder.buildDoubleSolenoidLogItems(wrist)),
                new DeviceLogger<DoubleSolenoid>(pincer, "Pincer",
                    LogProfileBuilder.buildDoubleSolenoidLogItems(pincer))
            }
        ));
    }

    /** Sets wrist to up position */
    public void setWristUp() {
        wrist.set(Value.kForward); // untested
    }

    /** Sets wrist to down position */
    public void setWristDown() {
        wrist.set(Value.kReverse); // untested
    }

    public void setPincerUp() {
        pincer.set(Value.kForward); // untested
    }

    public void setPincerDown() {
        pincer.set(Value.kReverse); // untested
    }

    /** 
     * Detects whether infrared beam is broken by pole
     * 
     * @return true if IR beam broken by pole
     */
    public boolean poleDetected() {
        return !poleDetector.get(); // untested
    }
}
