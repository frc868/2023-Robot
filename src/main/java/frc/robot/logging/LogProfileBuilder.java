package frc.robot.logging;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * A helper class that will automatically create an array of {@link LogItem}s
 * based on the type of object. Logs useful information about each device.
 * Reduces verbosity by a TON.
 * 
 * @author dr
 */
public class LogProfileBuilder {

    /**
     * Builds CANSparkMax log items. These include:
     * 
     * Encoder Distance,
     * Encoder Distance Conversion Factor,
     * Encoder Speed,
     * Encoder Speed Conversion Factor,
     * Speed,
     * Bus Voltage,
     * Temperature,
     * Output Current,
     * Device ID,
     * Firmware Version,
     * 
     * @param obj the CANSparkMax object to use
     * @return the array of LogItems
     */
    public static LogItem<?>[] buildCANSparkMaxLogItems(CANSparkMax obj) {
        return new LogItem<?>[] {
                new LogItem<Double>(LogType.NUMBER, "Encoder Distance", obj.getEncoder()::getPosition),
                new LogItem<Double>(LogType.NUMBER, "Encoder Distance Conversion Factor",
                        obj.getEncoder()::getPositionConversionFactor),
                new LogItem<Double>(LogType.NUMBER, "Encoder Speed", obj.getEncoder()::getVelocity),
                new LogItem<Double>(LogType.NUMBER, "Encoder Speed Conversion Factor",
                        obj.getEncoder()::getVelocityConversionFactor),
                new LogItem<Double>(LogType.NUMBER, "Speed", obj::get),
                new LogItem<Double>(LogType.NUMBER, "Bus Voltage", obj::getBusVoltage),
                new LogItem<Double>(LogType.NUMBER, "Temperature", obj::getMotorTemperature),
                new LogItem<Double>(LogType.NUMBER, "Output Current", obj::getOutputCurrent),
                new LogItem<Double>(LogType.NUMBER, "Device ID", () -> (double) obj.getDeviceId()),
                new LogItem<String>(LogType.STRING, "Firmware Version", obj::getFirmwareString),
        };
    }

    /**
     * Builds navX log items. These include:
     * 
     * Pitch,
     * Roll,
     * Yaw,
     * Angle,
     * Angle Rotation Rate,
     * X Axis Acceleration,
     * Y Axis Acceleration,
     * Z Axis Acceleration,
     * Compass Heading,
     * Is Calibrating,
     * Is Magnetometer Calibrated,
     * Is Connected,
     * Is Moving,
     * Is Rotating,
     * Is Magnetic Disturbance,
     * Temperature,
     * Update Count,
     * Firmware Version,
     * 
     * @param obj the navx to use
     * @return the array of LogItems
     */
    public static LogItem<?>[] buildNavXLogItems(AHRS obj) {
        return new LogItem<?>[] {
                new LogItem<Double>(LogType.NUMBER, "Pitch", () -> (double) obj.getPitch()),
                new LogItem<Double>(LogType.NUMBER, "Roll", () -> (double) obj.getRoll()),
                new LogItem<Double>(LogType.NUMBER, "Yaw", () -> (double) obj.getYaw()),
                new LogItem<Double>(LogType.NUMBER, "Angle", () -> (double) obj.getPitch()),
                new LogItem<Double>(LogType.NUMBER, "Angle Rotation Rate", obj::getRate),
                new LogItem<Double>(LogType.NUMBER, "X Axis Acceleration", () -> (double) obj.getWorldLinearAccelX()),
                new LogItem<Double>(LogType.NUMBER, "Y Axis Acceleration", () -> (double) obj.getWorldLinearAccelY()),
                new LogItem<Double>(LogType.NUMBER, "Z Axis Acceleration", () -> (double) obj.getWorldLinearAccelZ()),
                new LogItem<Double>(LogType.NUMBER, "Compass Heading", () -> (double) obj.getCompassHeading()),
                new LogItem<Boolean>(LogType.BOOLEAN, "Is Calibrating", obj::isCalibrating),
                new LogItem<Boolean>(LogType.BOOLEAN, "Is Magnetometer Calibrated", obj::isMagnetometerCalibrated),
                new LogItem<Boolean>(LogType.BOOLEAN, "Is Connected", obj::isConnected),
                new LogItem<Boolean>(LogType.BOOLEAN, "Is Moving", obj::isMoving),
                new LogItem<Boolean>(LogType.BOOLEAN, "Is Rotating", obj::isRotating),
                new LogItem<Boolean>(LogType.BOOLEAN, "Is Magnetic Disturbance", obj::isMagneticDisturbance),
                new LogItem<Double>(LogType.NUMBER, "Temperature", () -> (double) obj.getTempC()),
                new LogItem<Double>(LogType.NUMBER, "Update Count", obj::getUpdateCount),
                new LogItem<String>(LogType.STRING, "Firmware Version", obj::getFirmwareVersion)

        };
    }

    /**
     * Builds DoubleSolenoid log items. These include:
     * 
     * Position (true for forward),
     * Forward Channel,
     * Reverse Channel,
     * Is Forward Solenoid Disabled,
     * Is Reverse Solenoid Disabled,
     * 
     * @param obj the {@link DoubleSolenoid} to use
     * @return the array of LogItems
     */
    public static LogItem<?>[] buildDoubleSolenoidLogItems(DoubleSolenoid obj) {
        return new LogItem<?>[] {
                new LogItem<Boolean>(LogType.BOOLEAN, "Position", () -> obj.get() == DoubleSolenoid.Value.kForward),
                new LogItem<Double>(LogType.NUMBER, "Forward Channel", () -> (double) obj.getFwdChannel()),
                new LogItem<Double>(LogType.NUMBER, "Reverse Channel", () -> (double) obj.getRevChannel()),
                new LogItem<Boolean>(LogType.BOOLEAN, "Is Forward Solenoid Disabled", obj::isFwdSolenoidDisabled),
                new LogItem<Boolean>(LogType.BOOLEAN, "Is Reverse Solenoid Disabled", obj::isRevSolenoidDisabled)
        };
    }
}
