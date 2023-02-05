package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.techhounds.houndutil.houndlog.LogGroup;
import com.techhounds.houndutil.houndlog.LogProfileBuilder;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.loggers.DeviceLogger;

/**
 * The "subsystem" that contains objects that don't belong anywhere else but
 * need to have values logged for.
 * 
 * @author dr
 */
public class Misc extends SubsystemBase {
    /** The PDH (CAN ID 1) */
    private PowerDistribution pdh = new PowerDistribution();
    /** The PH (CAN ID 1) */
    private PneumaticHub ph = new PneumaticHub();

    /** Adds the loggers for the Miscellaneous group. */
    public Misc() {
        ph.enableCompressorDigital();
        LoggingManager.getInstance().addGroup("Miscellaneous", new LogGroup(
                new DeviceLogger<PowerDistribution>(pdh, "Power Distribution Hub",
                        LogProfileBuilder.buildPDHLogItems(pdh)),
                new DeviceLogger<PneumaticHub>(ph, "Pneumatic Hub",
                        LogProfileBuilder.buildPneumaticHubLogItems(ph))));
    }

}
