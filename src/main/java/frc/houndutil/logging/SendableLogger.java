package frc.houndutil.logging;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Basically a SingleItemLogger but it uses sendables. Useful for putting
 * anything Sendable like a Field2d, PIDController, or a motor controller.
 * 
 * @author dr
 * @apiNote do not put this or a LogGroup of this in a {@code periodic} method,
 *          since Sendables are sent to SmartDashboard declaratively. put these
 *          in an {@code init} method.
 */
public class SendableLogger extends Logger<Sendable> {
    private String key;
    private Sendable sendable;

    public SendableLogger(String subsystem, String key, Sendable sendable) {
        super(sendable, subsystem);
        this.key = key;
        this.sendable = sendable;
    }

    public SendableLogger(String key, Sendable sendable) {
        super(sendable, "Not set");
        this.key = key;
        this.sendable = sendable;
    }

    @Override
    public void init() {
        SmartDashboard.putData(subsystem + "/" + key, sendable);
    }

    @Override
    public void run() {

    }
}
