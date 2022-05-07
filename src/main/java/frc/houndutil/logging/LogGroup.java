package frc.houndutil.logging;

/**
 * Defines a group of {@link Logger}s to log. This is useful when used in a
 * subsystem where you only want to call one {@code logger.run()} method.
 * 
 * @author dr
 */
public class LogGroup {
    private Logger<?>[] loggers;

    public LogGroup(Logger<?>[] loggers) {
        this.loggers = loggers;
    }

    public LogGroup(String subsystem, Logger<?>[] loggers) {
        this.loggers = loggers;
        for (Logger<?> logger : loggers) {
            logger.setSubsystem(subsystem);
        }
    }

    public void init() {
        for (Logger<?> logger : loggers) {
            logger.init();
        }
    }

    public void run() {
        for (Logger<?> logger : loggers) {
            logger.run();
        }
    }
}
