package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.houndutil.houndlog.LogGroup;
import frc.houndutil.houndlog.LogProfileBuilder;
import frc.houndutil.houndlog.Logger;
import frc.houndutil.houndlog.DeviceLogger;

public class Misc extends SubsystemBase {
    private PowerDistribution pdh = new PowerDistribution();
    private PneumaticHub ph = new PneumaticHub();

    private LogGroup logger = new LogGroup("Miscellaneous", new Logger[] {
            new DeviceLogger<PowerDistribution>(pdh, "Power Distribution Hub",
                    LogProfileBuilder.buildPDHLogItems(pdh)),
            new DeviceLogger<PneumaticHub>(ph, "Pneumatic Hub",
                    LogProfileBuilder.buildPneumaticHubLogItems(ph)),
    });

    public Misc() {

    }

    /**
     * Runs every 20ms. In this method, all we do is run SmartDashboard/logging
     * related functions (do NOT run any code that should belong in a command here!)
     */
    @Override
    public void periodic() {
        logger.run();
    }

}
