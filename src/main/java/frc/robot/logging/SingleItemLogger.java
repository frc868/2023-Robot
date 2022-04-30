package frc.robot.logging;

import java.util.concurrent.Callable;

/**
 * A logger with a single LogItem. Useful for subsystems that need to catalogue
 * one value (like a PID setpoint, perhaps).
 * 
 * @author dr
 */
public class SingleItemLogger<T> extends Logger<T> {
    public SingleItemLogger(String subsystem, LogType type, String key, Callable<T> func) {
        super(null, subsystem, new LogItem<?>[] { new LogItem<T>(type, key, func) });
        // here, T is the same between both Logger and LogItem since Logger basically
        // just acts as a wrapper, so the T can be used for both. kinda hacky, but it
        // works.
    }

    public SingleItemLogger(LogType type, String key, Callable<T> func) {
        this("Not set", type, key, func);
    }
}
