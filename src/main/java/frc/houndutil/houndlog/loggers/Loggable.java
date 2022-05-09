package frc.houndutil.houndlog.loggers;

/**
 * Defines a loggable. This was mainly necessary due to wanting to include both
 * {@link Logger}s and {@link LogGroup}s in {@link LoggingManager}.
 * 
 * @author dr
 */
public interface Loggable {
    public void init();

    public void run();
}
