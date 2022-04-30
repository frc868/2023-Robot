package frc.robot.logging;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
     * Runs the logger. This will call the functions listed to log and convert them
     * to the correct type.
     */
    public void run() {
        for (LogItem<?> v : values) {
            try {
                switch (v.getType()) {
                    case STRING:
                        SmartDashboard.putString(subsystem + "/" + deviceName + "/" + v.getKey(),
                                (String) v.getFunc().call());
                        break;
                    case NUMBER:
                        SmartDashboard.putNumber(subsystem + "/" + deviceName + "/" + v.getKey(),
                                (double) v.getFunc().call());
                        break;
                    case BOOLEAN:
                        SmartDashboard.putBoolean(subsystem + "/" + deviceName + "/" + v.getKey(),
                                (boolean) v.getFunc().call());
                        break;
                    case DATA:
                        SmartDashboard.putData(subsystem + "/" + deviceName + "/" + v.getKey(),
                                (Sendable) v.getFunc().call());
                        break;
                    case STRING_ARRAY:
                        SmartDashboard.putStringArray(subsystem + "/" + deviceName + "/" + v.getKey(),
                                (String[]) v.getFunc().call());
                        break;
                    case NUMBER_ARRAY:
                        SmartDashboard.putNumberArray(subsystem + "/" + deviceName + "/" + v.getKey(),
                                (double[]) v.getFunc().call());
                        break;
                    case BOOLEAN_ARRAY:
                        SmartDashboard.putBooleanArray(subsystem + "/" + deviceName + "/" + v.getKey(),
                                (boolean[]) v.getFunc().call());
                        break;
                    default:
                        SmartDashboard.putString(subsystem + "/" + deviceName + "/" + v.getKey(), "Unspecified type.");
                        break;
                }
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }
}
