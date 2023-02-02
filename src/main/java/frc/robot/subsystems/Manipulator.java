package frc.robot.subsystems;

import com.techhounds.houndutil.houndlog.LogGroup;
import com.techhounds.houndutil.houndlog.LogProfileBuilder;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.loggers.DeviceLogger;
import com.techhounds.houndutil.houndlog.loggers.Logger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
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
    private DoubleSolenoid wristSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.Manipulator.Solenoids.FORWARD, Constants.Manipulator.Solenoids.REVERSE);
    private DigitalInput poleDetector = new DigitalInput(Constants.Manipulator.POLE_DETECTOR);

    public Manipulator() {
        LoggingManager.getInstance().addGroup("Manipulator", new LogGroup(
            new Logger[] {
                new DeviceLogger<DoubleSolenoid>(wristSolenoid, "Wrist Solenoid",
                LogProfileBuilder.buildDoubleSolenoidLogItems(wristSolenoid))
            }
        ));
    }

    public void setWristUp() {
        wristSolenoid.set(Value.kForward); // untested
    }

    public void setWristDown() {
        wristSolenoid.set(Value.kReverse); // untested
    }

    public boolean poleDetected() {
        return poleDetector.get();
    }
}
