package frc.robot.sensors;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The Astra subsystem uses the Pi and the Orbbec Astra camera to get
 * information on where balls are on the field. This can be used to target and
 * turn to balls in both teleop and auton.
 * 
 * @author dr
 */
public class Astra extends SubsystemBase {
    private static NetworkTable table = NetworkTableInstance.getDefault().getTable("HoundEye");;

    public Astra() {
    }

    /**
     * Get the alliance that the Pi thinks we are (mainly for debug)
     * 
     * @return either "B" or "R"
     */
    public String getAlliance() {
        NetworkTableEntry alliance = table.getEntry("alliance");
        return alliance.getString("");
    }

    /**
     * Get the x offset from a specified ball in degrees.
     * Balls are sorted by distance (0 is the closest).
     * This will only return balls that are our current
     * alliance (that can be set in the Driver Station).
     * 
     * @param ball_number ball to provide information for
     * @return the x offset in degrees
     */
    public double getTx(int ball_number) {
        NetworkTableEntry tx = table.getEntry("tx");
        double[] arr = tx.getDoubleArray(new double[] { 0.0 }); // 0.0 default if value doesn't exist
        if (arr.length > 0) {
            return arr[0];
        } else {
            return 0.0;
        }
    }

    /**
     * Get the y offset from a specified ball in degrees.
     * Balls are sorted by distance (0 is the closest).
     * This will only return balls that are our current
     * alliance (that can be set in the Driver Station).
     * 
     * @param ball_number ball to provide information for
     * @return the y offset in degrees
     */
    public double getTy(int ball_number) {
        NetworkTableEntry ty = table.getEntry("ty");
        double[] arr = ty.getDoubleArray(new double[] { 0.0 }); // 0.0 default if value doesn't exist
        if (arr.length > 0) {
            return arr[0];
        } else {
            return 0.0;
        }
    }

    /**
     * Get the distance from a specified ball in inches.
     * Balls are sorted by distance (0 is the closest)
     * This will only return balls that are our current
     * alliance (that can be set in the Driver Station).
     * 
     * @param ball_number ball to provide information for
     * @return the distance in inches
     * @author dr
     */
    public double getTd(int ball_number) {
        NetworkTableEntry td = table.getEntry("td");
        double[] arr = td.getDoubleArray(new double[] { 0.0 }); // 0.0 default if value doesn't exist
        if (arr.length > 0) {
            return arr[0];
        } else {
            return 0.0;
        }
    }
}
