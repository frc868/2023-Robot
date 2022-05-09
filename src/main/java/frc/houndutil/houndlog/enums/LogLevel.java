package frc.houndutil.houndlog.enums;

/**
 * The type of Logger to create. These are values defined by NetworkTables.
 * 
 * @author dr
 */
public enum LogLevel {
    DEBUG, // this will only run when debug mode is on
    INFO, // this will only run when the bot is enabled in test mode
    MAIN // this will run all the time
}
