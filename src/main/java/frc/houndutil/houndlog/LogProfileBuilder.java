package frc.houndutil.houndlog;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

/**
 * A helper class that will automatically create an array of {@link LogItem}s
 * based on the type of object. Logs useful information about each device.
 * Reduces verbosity by a TON.
 * 
 * @author dr
 */
public class LogProfileBuilder {

    /**
     * Builds CANSparkMax log items.
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
                new LogItem<Double>(LogType.NUMBER, "Closed Loop Ramp Rate", obj::getClosedLoopRampRate),
                new LogItem<Double>(LogType.NUMBER, "Open Loop Ramp Rate", obj::getOpenLoopRampRate),
                new LogItem<Double>(LogType.NUMBER, "Speed", obj::get),
                new LogItem<Double>(LogType.NUMBER, "Bus Voltage", obj::getBusVoltage),
                new LogItem<Double>(LogType.NUMBER, "Motor Temperature", obj::getMotorTemperature),
                new LogItem<Double>(LogType.NUMBER, "Output Current", obj::getOutputCurrent),
                new LogItem<Double>(LogType.NUMBER, "Device ID", () -> (double) obj.getDeviceId()),
                new LogItem<String>(LogType.STRING, "Firmware Version", obj::getFirmwareString),
                new LogItem<Boolean>(LogType.BOOLEAN, "Is In Brake Mode",
                        () -> obj.getIdleMode() == CANSparkMax.IdleMode.kBrake),
                new LogItem<Boolean>(LogType.BOOLEAN, "Is Inverted", obj::getInverted),
                new LogItem<Boolean>(LogType.BOOLEAN, "Is Follower", obj::isFollower),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/Brownout",
                        () -> obj.getFault(CANSparkMax.FaultID.kBrownout)),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/Has Reset",
                        () -> obj.getFault(CANSparkMax.FaultID.kHasReset)),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/Motor Fault",
                        () -> obj.getFault(CANSparkMax.FaultID.kMotorFault)),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/Other Fault",
                        () -> obj.getFault(CANSparkMax.FaultID.kOtherFault)),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/Overcurrent",
                        () -> obj.getFault(CANSparkMax.FaultID.kOvercurrent)),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/Sensor Fault",
                        () -> obj.getFault(CANSparkMax.FaultID.kSensorFault)),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/Stalled", () -> obj.getFault(CANSparkMax.FaultID.kStall)),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/DRV Fault",
                        () -> obj.getFault(CANSparkMax.FaultID.kStall)),
                new LogItem<Boolean>(LogType.BOOLEAN, "Sticky Faults/Brownout",
                        () -> obj.getStickyFault(CANSparkMax.FaultID.kBrownout)),
                new LogItem<Boolean>(LogType.BOOLEAN, "Sticky Faults/Has Reset",
                        () -> obj.getStickyFault(CANSparkMax.FaultID.kHasReset)),
                new LogItem<Boolean>(LogType.BOOLEAN, "Sticky Faults/Motor Fault",
                        () -> obj.getStickyFault(CANSparkMax.FaultID.kMotorFault)),
                new LogItem<Boolean>(LogType.BOOLEAN, "Sticky Faults/Other Fault",
                        () -> obj.getStickyFault(CANSparkMax.FaultID.kOtherFault)),
                new LogItem<Boolean>(LogType.BOOLEAN, "Sticky Faults/Overcurrent",
                        () -> obj.getStickyFault(CANSparkMax.FaultID.kOvercurrent)),
                new LogItem<Boolean>(LogType.BOOLEAN, "Sticky Faults/Sensor Fault",
                        () -> obj.getStickyFault(CANSparkMax.FaultID.kSensorFault)),
                new LogItem<Boolean>(LogType.BOOLEAN, "Sticky Faults/Stalled",
                        () -> obj.getStickyFault(CANSparkMax.FaultID.kStall)),
                new LogItem<Boolean>(LogType.BOOLEAN, "Sticky Faults/DRV Fault",
                        () -> obj.getStickyFault(CANSparkMax.FaultID.kStall)),
        };
    }

    /**
     * Builds navX log items.
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
     * Builds DoubleSolenoid log items.
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

    /**
     * Builds PDH log items.
     * 
     * @param obj the PDH instance to use
     * @return the array of LogItems
     */
    public static LogItem<?>[] buildPDHLogItems(PowerDistribution obj) {
        return new LogItem<?>[] {
                new LogItem<Double>(LogType.NUMBER, "Voltage", obj::getVoltage),
                new LogItem<Double>(LogType.NUMBER, "Temperature", obj::getTemperature),
                new LogItem<Double>(LogType.NUMBER, "Total Current (A)", obj::getTotalCurrent),
                new LogItem<Double>(LogType.NUMBER, "Total Power (W)", obj::getTotalPower),
                new LogItem<Double>(LogType.NUMBER, "Total Energy (J)", obj::getTotalEnergy),
                new LogItem<Double>(LogType.NUMBER, "Channel 0 Current", () -> obj.getCurrent(0)),
                new LogItem<Double>(LogType.NUMBER, "Channel 1 Current", () -> obj.getCurrent(1)),
                new LogItem<Double>(LogType.NUMBER, "Channel 2 Current", () -> obj.getCurrent(2)),
                new LogItem<Double>(LogType.NUMBER, "Channel 3 Current", () -> obj.getCurrent(3)),
                new LogItem<Double>(LogType.NUMBER, "Channel 4 Current", () -> obj.getCurrent(4)),
                new LogItem<Double>(LogType.NUMBER, "Channel 5 Current", () -> obj.getCurrent(5)),
                new LogItem<Double>(LogType.NUMBER, "Channel 6 Current", () -> obj.getCurrent(6)),
                new LogItem<Double>(LogType.NUMBER, "Channel 7 Current", () -> obj.getCurrent(7)),
                new LogItem<Double>(LogType.NUMBER, "Channel 8 Current", () -> obj.getCurrent(8)),
                new LogItem<Double>(LogType.NUMBER, "Channel 9 Current", () -> obj.getCurrent(9)),
                new LogItem<Double>(LogType.NUMBER, "Channel 10 Current", () -> obj.getCurrent(10)),
                new LogItem<Double>(LogType.NUMBER, "Channel 11 Current", () -> obj.getCurrent(11)),
                new LogItem<Double>(LogType.NUMBER, "Channel 12 Current", () -> obj.getCurrent(12)),
                new LogItem<Double>(LogType.NUMBER, "Channel 13 Current", () -> obj.getCurrent(13)),
                new LogItem<Double>(LogType.NUMBER, "Channel 14 Current", () -> obj.getCurrent(14)),
                new LogItem<Double>(LogType.NUMBER, "Channel 15 Current", () -> obj.getCurrent(15)),
                new LogItem<Double>(LogType.NUMBER, "Channel 16 Current", () -> obj.getCurrent(16)),
                new LogItem<Double>(LogType.NUMBER, "Channel 17 Current", () -> obj.getCurrent(17)),
                new LogItem<Double>(LogType.NUMBER, "Channel 18 Current", () -> obj.getCurrent(18)),
                new LogItem<Double>(LogType.NUMBER, "Channel 19 Current", () -> obj.getCurrent(19)),
                new LogItem<Double>(LogType.NUMBER, "Channel 20 Current", () -> obj.getCurrent(20)),
                new LogItem<Double>(LogType.NUMBER, "Channel 21 Current", () -> obj.getCurrent(21)),
                new LogItem<Double>(LogType.NUMBER, "Channel 22 Current", () -> obj.getCurrent(22)),
                new LogItem<Double>(LogType.NUMBER, "Channel 23 Current", () -> obj.getCurrent(23)),
                new LogItem<Double>(LogType.NUMBER, "Version/Firmware Major",
                        () -> (double) obj.getVersion().firmwareMajor),
                new LogItem<Double>(LogType.NUMBER, "Version/Firmware Minor",
                        () -> (double) obj.getVersion().firmwareMinor),
                new LogItem<Double>(LogType.NUMBER, "Version/Firmware Fix",
                        () -> (double) obj.getVersion().firmwareFix),
                new LogItem<Double>(LogType.NUMBER, "Version/Hardware Major",
                        () -> (double) obj.getVersion().hardwareMajor),
                new LogItem<Double>(LogType.NUMBER, "Version/Hardware Minor",
                        () -> (double) obj.getVersion().hardwareMinor),
                new LogItem<Double>(LogType.NUMBER, "Unique ID", () -> (double) obj.getVersion().uniqueId),
                new LogItem<String>(LogType.STRING, "Type",
                        () -> obj.getType() == ModuleType.kRev ? "Rev PDH" : "CTRE PDP"),
                new LogItem<Boolean>(LogType.BOOLEAN, "Is Switchable Channel On", obj::getSwitchableChannel),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/Channel 0 Breaker Fault",
                        () -> obj.getFaults().Channel0BreakerFault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/Channel 1 Breaker Fault",
                        () -> obj.getFaults().Channel1BreakerFault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/Channel 2 Breaker Fault",
                        () -> obj.getFaults().Channel2BreakerFault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/Channel 3 Breaker Fault",
                        () -> obj.getFaults().Channel3BreakerFault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/Channel 4 Breaker Fault",
                        () -> obj.getFaults().Channel4BreakerFault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/Channel 5 Breaker Fault",
                        () -> obj.getFaults().Channel5BreakerFault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/Channel 6 Breaker Fault",
                        () -> obj.getFaults().Channel6BreakerFault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/Channel 7 Breaker Fault",
                        () -> obj.getFaults().Channel7BreakerFault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/Channel 8 Breaker Fault",
                        () -> obj.getFaults().Channel8BreakerFault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/Channel 9 Breaker Fault",
                        () -> obj.getFaults().Channel9BreakerFault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/Channel 10 Breaker Fault",
                        () -> obj.getFaults().Channel11BreakerFault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/Channel 11 Breaker Fault",
                        () -> obj.getFaults().Channel10BreakerFault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/Channel 12 Breaker Fault",
                        () -> obj.getFaults().Channel12BreakerFault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/Channel 13 Breaker Fault",
                        () -> obj.getFaults().Channel13BreakerFault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/Channel 14 Breaker Fault",
                        () -> obj.getFaults().Channel14BreakerFault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/Channel 15 Breaker Fault",
                        () -> obj.getFaults().Channel15BreakerFault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/Channel 16 Breaker Fault",
                        () -> obj.getFaults().Channel16BreakerFault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/Channel 17 Breaker Fault",
                        () -> obj.getFaults().Channel17BreakerFault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/Channel 18 Breaker Fault",
                        () -> obj.getFaults().Channel18BreakerFault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/Channel 19 Breaker Fault",
                        () -> obj.getFaults().Channel19BreakerFault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/Channel 20 Breaker Fault",
                        () -> obj.getFaults().Channel20BreakerFault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/Channel 21 Breaker Fault",
                        () -> obj.getFaults().Channel21BreakerFault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/Channel 22 Breaker Fault",
                        () -> obj.getFaults().Channel22BreakerFault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/Channel 23 Breaker Fault",
                        () -> obj.getFaults().Channel23BreakerFault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/Brownout", () -> obj.getFaults().Brownout),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/CAN Warning",
                        () -> obj.getFaults().CanWarning),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/Hardware Fault", () -> obj.getFaults().HardwareFault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Sticky Faults/Channel 0 Breaker Fault",
                        () -> obj.getStickyFaults().Channel0BreakerFault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Sticky Faults/Channel 1 Breaker Fault",
                        () -> obj.getStickyFaults().Channel1BreakerFault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Sticky Faults/Channel 2 Breaker Fault",
                        () -> obj.getStickyFaults().Channel2BreakerFault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Sticky Faults/Channel 3 Breaker Fault",
                        () -> obj.getStickyFaults().Channel3BreakerFault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Sticky Faults/Channel 4 Breaker Fault",
                        () -> obj.getStickyFaults().Channel4BreakerFault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Sticky Faults/Channel 5 Breaker Fault",
                        () -> obj.getStickyFaults().Channel5BreakerFault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Sticky Faults/Channel 6 Breaker Fault",
                        () -> obj.getStickyFaults().Channel6BreakerFault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Sticky Faults/Channel 7 Breaker Fault",
                        () -> obj.getStickyFaults().Channel7BreakerFault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Sticky Faults/Channel 8 Breaker Fault",
                        () -> obj.getStickyFaults().Channel8BreakerFault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Sticky Faults/Channel 9 Breaker Fault",
                        () -> obj.getStickyFaults().Channel9BreakerFault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Sticky Faults/Channel 10 Breaker Fault",
                        () -> obj.getStickyFaults().Channel11BreakerFault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Sticky Faults/Channel 11 Breaker Fault",
                        () -> obj.getStickyFaults().Channel10BreakerFault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Sticky Faults/Channel 12 Breaker Fault",
                        () -> obj.getStickyFaults().Channel12BreakerFault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Sticky Faults/Channel 13 Breaker Fault",
                        () -> obj.getStickyFaults().Channel13BreakerFault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Sticky Faults/Channel 14 Breaker Fault",
                        () -> obj.getStickyFaults().Channel14BreakerFault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Sticky Faults/Channel 15 Breaker Fault",
                        () -> obj.getStickyFaults().Channel15BreakerFault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Sticky Faults/Channel 16 Breaker Fault",
                        () -> obj.getStickyFaults().Channel16BreakerFault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Sticky Faults/Channel 17 Breaker Fault",
                        () -> obj.getStickyFaults().Channel17BreakerFault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Sticky Faults/Channel 18 Breaker Fault",
                        () -> obj.getStickyFaults().Channel18BreakerFault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Sticky Faults/Channel 19 Breaker Fault",
                        () -> obj.getStickyFaults().Channel19BreakerFault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Sticky Faults/Channel 20 Breaker Fault",
                        () -> obj.getStickyFaults().Channel20BreakerFault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Sticky Faults/Channel 21 Breaker Fault",
                        () -> obj.getStickyFaults().Channel21BreakerFault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Sticky Faults/Channel 22 Breaker Fault",
                        () -> obj.getStickyFaults().Channel22BreakerFault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Sticky Faults/Channel 23 Breaker Fault",
                        () -> obj.getStickyFaults().Channel23BreakerFault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Sticky Faults/Brownout", () -> obj.getStickyFaults().Brownout),
                new LogItem<Boolean>(LogType.BOOLEAN, "Sticky Faults/CAN Bus Off",
                        () -> obj.getStickyFaults().CanBusOff),
                new LogItem<Boolean>(LogType.BOOLEAN, "Sticky Faults/CAN Warning",
                        () -> obj.getStickyFaults().CanWarning),
                new LogItem<Boolean>(LogType.BOOLEAN, "Sticky Faults/Has Reset", () -> obj.getStickyFaults().HasReset),

        };
    }

    /**
     * Builds REV Pneumatics Hub log items.
     * 
     * @param obj the PH instance to use
     * @return the array of LogItems
     */
    public static LogItem<?>[] buildPneumaticHubLogItems(PneumaticHub obj) {
        return new LogItem<?>[] {
                new LogItem<Double>(LogType.NUMBER, "Input Voltage", obj::getInputVoltage),
                new LogItem<Double>(LogType.NUMBER, "5V Regulated Voltage", obj::get5VRegulatedVoltage),
                new LogItem<Double>(LogType.NUMBER, "Module Number", () -> (double) obj.getModuleNumber()),
                new LogItem<Double>(LogType.NUMBER, "Channel 0 Pressure", () -> obj.getPressure(0)),
                new LogItem<Double>(LogType.NUMBER, "Channel 1 Pressure", () -> obj.getPressure(1)),
                new LogItem<Double>(LogType.NUMBER, "Channel 2 Pressure", () -> obj.getPressure(2)),
                new LogItem<Double>(LogType.NUMBER, "Channel 3 Pressure", () -> obj.getPressure(3)),
                new LogItem<Double>(LogType.NUMBER, "Channel 4 Pressure", () -> obj.getPressure(4)),
                new LogItem<Double>(LogType.NUMBER, "Channel 5 Pressure", () -> obj.getPressure(5)),
                new LogItem<Double>(LogType.NUMBER, "Channel 6 Pressure", () -> obj.getPressure(6)),
                new LogItem<Double>(LogType.NUMBER, "Channel 7 Pressure", () -> obj.getPressure(7)),
                new LogItem<Double>(LogType.NUMBER, "Channel 8 Pressure", () -> obj.getPressure(8)),
                new LogItem<Double>(LogType.NUMBER, "Channel 9 Pressure", () -> obj.getPressure(9)),
                new LogItem<Double>(LogType.NUMBER, "Channel 10 Pressure", () -> obj.getPressure(10)),
                new LogItem<Double>(LogType.NUMBER, "Channel 11 Pressure", () -> obj.getPressure(11)),
                new LogItem<Double>(LogType.NUMBER, "Channel 12 Pressure", () -> obj.getPressure(12)),
                new LogItem<Double>(LogType.NUMBER, "Channel 13 Pressure", () -> obj.getPressure(13)),
                new LogItem<Double>(LogType.NUMBER, "Channel 14 Pressure", () -> obj.getPressure(14)),
                new LogItem<Double>(LogType.NUMBER, "Channel 15 Pressure", () -> obj.getPressure(15)),
                new LogItem<Boolean>(LogType.BOOLEAN, "Pressure Switch", obj::getPressureSwitch),
                new LogItem<Double>(LogType.NUMBER, "Compressor Current", obj::getCompressorCurrent),
                new LogItem<Boolean>(LogType.BOOLEAN, "Is Compressor Running", obj::getCompressor),
                new LogItem<Double>(LogType.NUMBER, "Solenoids", () -> (double) obj.getSolenoids()),
                new LogItem<Double>(LogType.NUMBER, "Solenoids Total Current", obj::getSolenoidsTotalCurrent),
                new LogItem<Double>(LogType.NUMBER, "Solenoids Voltage", obj::getSolenoidsVoltage),
                new LogItem<Double>(LogType.NUMBER, "Version/Firmware Major",
                        () -> (double) obj.getVersion().firmwareMajor),
                new LogItem<Double>(LogType.NUMBER, "Version/Firmware Minor",
                        () -> (double) obj.getVersion().firmwareMinor),
                new LogItem<Double>(LogType.NUMBER, "Version/Firmware Fix",
                        () -> (double) obj.getVersion().firmwareFix),
                new LogItem<Double>(LogType.NUMBER, "Version/Hardware Major",
                        () -> (double) obj.getVersion().hardwareMajor),
                new LogItem<Double>(LogType.NUMBER, "Version/Hardware Minor",
                        () -> (double) obj.getVersion().hardwareMinor),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/Brownout", () -> obj.getFaults().Brownout),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/CAN Warning", () -> obj.getFaults().CanWarning),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/Channel 0 Fault", () -> obj.getFaults().Channel0Fault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/Channel 1 Fault", () -> obj.getFaults().Channel1Fault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/Channel 2 Fault", () -> obj.getFaults().Channel2Fault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/Channel 3 Fault", () -> obj.getFaults().Channel3Fault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/Channel 4 Fault", () -> obj.getFaults().Channel4Fault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/Channel 5 Fault", () -> obj.getFaults().Channel5Fault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/Channel 6 Fault", () -> obj.getFaults().Channel6Fault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/Channel 7 Fault", () -> obj.getFaults().Channel7Fault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/Channel 8 Fault", () -> obj.getFaults().Channel8Fault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/Channel 9 Fault", () -> obj.getFaults().Channel9Fault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/Channel 10 Fault", () -> obj.getFaults().Channel10Fault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/Channel 11 Fault", () -> obj.getFaults().Channel11Fault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/Channel 12 Fault", () -> obj.getFaults().Channel12Fault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/Channel 13 Fault", () -> obj.getFaults().Channel13Fault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/Channel 14 Fault", () -> obj.getFaults().Channel14Fault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/Channel 15 Fault", () -> obj.getFaults().Channel15Fault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/Compressor Open", () -> obj.getFaults().CompressorOpen),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/Compressor Over Current",
                        () -> obj.getFaults().CompressorOverCurrent),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/Hardware Fault", () -> obj.getFaults().HardwareFault),
                new LogItem<Boolean>(LogType.BOOLEAN, "Faults/Solenoid Over Current",
                        () -> obj.getFaults().SolenoidOverCurrent),
                new LogItem<Boolean>(LogType.BOOLEAN, "Sticky Faults/Brownout", () -> obj.getStickyFaults().Brownout),
                new LogItem<Boolean>(LogType.BOOLEAN, "Sticky Faults/CAN Bus Off",
                        () -> obj.getStickyFaults().CanBusOff),
                new LogItem<Boolean>(LogType.BOOLEAN, "Sticky Faults/CAN Warning",
                        () -> obj.getStickyFaults().CanWarning),
                new LogItem<Boolean>(LogType.BOOLEAN, "Sticky Faults/Compressor Open",
                        () -> obj.getStickyFaults().CompressorOpen),
                new LogItem<Boolean>(LogType.BOOLEAN, "Sticky Faults/Compressor Over Current",
                        () -> obj.getStickyFaults().CompressorOverCurrent),
                new LogItem<Boolean>(LogType.BOOLEAN, "Sticky Faults/Has Reset", () -> obj.getStickyFaults().HasReset),
                new LogItem<Boolean>(LogType.BOOLEAN, "Sticky Faults/Solenoid Over Current",
                        () -> obj.getStickyFaults().SolenoidOverCurrent),

        };
    }

}
