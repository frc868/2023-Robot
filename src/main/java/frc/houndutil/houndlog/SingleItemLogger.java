package frc.houndutil.houndlog;

import java.util.concurrent.Callable;

import edu.wpi.first.networktables.NetworkTable;

/**
 * A logger with a single LogItem. Useful for subsystems that need to catalogue
 * one value (like a PID setpoint, perhaps).
 * 
 * @author dr
 */
public class SingleItemLogger<T> extends Logger {
    private LogItem<?> item;

    /**
     * Instantiates a SingleItemLogger.
     * 
     * @param subsystem the name of the subsystem
     * @param type      the type of log
     * @param key       the key of the log entry
     * @param func      the function that will provide the value for the log entry
     */
    public SingleItemLogger(String subsystem, LogType type, String key, Callable<T> func) {
        super(subsystem);
        item = new LogItem<T>(type, key, func);
    }

    /**
     * Instantiates a SingleItemLogger (without a subsystem, only use if setting
     * the subsystem through LogGroup).
     * 
     * @param type the type of log
     * @param key  the key of the log entry
     * @param func the function that will provide the value for the log entry
     */
    public SingleItemLogger(LogType type, String key, Callable<T> func) {
        item = new LogItem<T>(type, key, func);
    }

    /**
     * Gets the subsystem table.
     */
    @Override
    protected NetworkTable getDataTable() {
        return getLogTable().getSubTable(subsystem);
    }

    /**
     * Does nothing, because we don't need to init for a {@code SingleItemLogger}.
     */
    @Override
    public void init() {

    }

    /**
     * Logs the single item stored.
     */
    @Override
    public void run() {
        logItem(item);
    }

}
