package frc.houndutil.houndlog;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
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

    /**
     * This is required because I'm stubborn and don't want to use SmartDashboard
     * for this, so I had to copy the code to send a Sendable over NetworkTables
     * from {@link SmartDashboard}.
     * 
     * @param s        the sendable to send
     * @param logTable the table through which to send it
     */
    private void publishSendable(NetworkTable logTable) {
        NetworkTable dataTable = logTable.getSubTable(subsystem).getSubTable(key);
        SendableBuilderImpl builder = new SendableBuilderImpl();
        builder.setTable(dataTable);
        SendableRegistry.publish(sendable, builder);
        builder.startListeners();
        dataTable.getEntry(".name").setString(key);
    }

    @Override
    public void init() {
        NetworkTable logTable = NetworkTableInstance.getDefault().getTable("HoundLog");
        publishSendable(logTable);
    }

    @Override
    public void run() {

    }
}
