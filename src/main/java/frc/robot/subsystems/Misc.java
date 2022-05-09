package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.houndutil.houndlog.LogGroup;
import frc.houndutil.houndlog.LogProfileBuilder;
import frc.houndutil.houndlog.LoggingManager;
import frc.houndutil.houndlog.loggers.DeviceLogger;
import frc.houndutil.houndlog.loggers.Logger;

public class Misc extends SubsystemBase {
    private PowerDistribution pdh = new PowerDistribution();
    private PneumaticHub ph = new PneumaticHub();

    public Misc() {
        LoggingManager.getInstance().addGroup("Miscellaneous", new LogGroup(
                new Logger[] {
                        new DeviceLogger<PowerDistribution>(pdh, "Power Distribution Hub",
                                LogProfileBuilder.buildPDHLogItems(pdh)),
                        new DeviceLogger<PneumaticHub>(ph, "Pneumatic Hub",
                                LogProfileBuilder.buildPneumaticHubLogItems(ph)),
                }));
    }

}
