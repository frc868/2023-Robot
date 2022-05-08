package frc.houndutil.houndlog;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * A logger for a specified object T. This logger will post all values contained
 * in {@code values} to SmartDashboard.
 * 
 * The group of logging classes under {@code frc.robot.logging} is designed to
 * be dropped straight into any
 * robot project, not just this one.
 * 
 * @author dr
 */
public class Logger<T> {
    /**
     * The object we are logging.
     */
    public T obj;

    /**
     * The name of the subsystem to log under (the naming convention in SD is
     * SmartDashboard/{subsystem}/{device_name}/{value_name}).
     */
    protected String subsystem;

    /**
     * The name of the device to log.
     */
    private String deviceName;

    /**
     * An array of values to log. Using the unspecified generic form since this list
     * contains several types of {@code LogItem}.
     */
    private LogItem<?>[] values;

    /**
     * Constructs a Logger object.
     * 
     * @param obj        the object to log values for
     * @param subsystem  the name of the subsystem
     * @param deviceName the name of the device
     * @param values     the list of values to log (can be created manually or
     *                   through {@link LogProfileBuilder})
     */
    public Logger(T obj, String subsystem, String deviceName, LogItem<?>[] values) {
        this.obj = obj;
        this.subsystem = subsystem;
        this.deviceName = deviceName;
        this.values = values;
    }

    /**
     * Constructs a Logger object (without a subsystem, only use if setting the
     * subsystem through LogGroup).
     * 
     * @param obj        the object to log values for
     * @param deviceName the name of the device
     * @param values     the list of values to log (can be created manually or
     *                   through {@link LogProfileBuilder})
     */
    public Logger(T obj, String deviceName, LogItem<?>[] values) {
        this.obj = obj;
        this.subsystem = "Not set";
        this.deviceName = deviceName;
        this.values = values;
    }

    /**
     * A more minimal constructor so {@link SendableLogger} can work better. This is
     * protected so that it can only be used by subclasses (like
     * {@link SendableLogger})
     * 
     * @param obj       the (Sendable) object to log values for
     * @param subsystem the name of the subsystem
     */
    protected Logger(T obj, String subsystem) {
        this.obj = obj;
        this.subsystem = subsystem;
        this.deviceName = "";
        this.values = new LogItem<?>[] {}; // an empty array so that run() doesn't fail if this is used incorrectly
    }

    public void setSubsystem(String subsystem) {
        this.subsystem = subsystem;
    }

    /**
     * Inits the logger. Used for SendableLoggers. Intended to be overridden.
     */
    public void init() {
    }

    /**
     * Runs the logger. This will call the functions listed to log and convert them
     * to the correct type.
     */
    public void run() {
        NetworkTable logTable = NetworkTableInstance.getDefault().getTable("HoundLog");
        for (LogItem<?> v : values) {
            try {
                switch (v.getType()) {
                    case STRING:
                        logTable.getSubTable(subsystem).getSubTable(deviceName).getEntry(v.getKey())
                                .setString((String) v.getFunc().call());
                        break;
                    case NUMBER:
                        logTable.getSubTable(subsystem).getSubTable(deviceName).getEntry(v.getKey())
                                .setDouble((double) v.getFunc().call());
                        break;
                    case BOOLEAN:
                        logTable.getSubTable(subsystem).getSubTable(deviceName).getEntry(v.getKey())
                                .setBoolean((boolean) v.getFunc().call());
                        break;
                    case STRING_ARRAY:
                        logTable.getSubTable(subsystem).getSubTable(deviceName).getEntry(v.getKey())
                                .setStringArray((String[]) v.getFunc().call());
                        break;
                    case NUMBER_ARRAY:
                        logTable.getSubTable(subsystem).getSubTable(deviceName).getEntry(v.getKey())
                                .setDoubleArray((double[]) v.getFunc().call());
                        break;
                    case BOOLEAN_ARRAY:
                        logTable.getSubTable(subsystem).getSubTable(deviceName).getEntry(v.getKey())
                                .setBooleanArray((boolean[]) v.getFunc().call());
                        break;
                    default:
                        logTable.getSubTable(subsystem).getSubTable(deviceName).getEntry(v.getKey())
                                .setString("Unspecified type.");
                        break;
                }
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }
}
