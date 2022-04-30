package frc.robot.sensors;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * The wrapper for receiving limelight data.
 * 
 * @author dr
 */
public class Limelight extends SubsystemBase {
    private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    public Limelight() {

    }

    /**
     * Get whether or not the Limelight has any valid targets (0 or 1)
     * 
     * @return
     */
    public double getTv() {
        NetworkTableEntry tv = table.getEntry("tv");
        return tv.getDouble(0.0);
    }

    /**
     * Get the x offset from the target in degrees
     * 
     * @author dr
     */
    public double getTx() {
        NetworkTableEntry tx = table.getEntry("tx");
        return tx.getDouble(0.0);
    }

    /**
     * Get the y offset from the target in degrees
     * 
     * @author dr
     */
    public double getTy() {
        NetworkTableEntry ty = table.getEntry("ty");
        return ty.getDouble(0.0);
    }

    /**
     * Get the percentage the target takes
     * of the frame (can be used to calc distance)
     * 
     * @author dr
     */
    public double getTa() {
        NetworkTableEntry ta = table.getEntry("ta");
        return ta.getDouble(0.0);
    }

    /**
     * Get the distance the limelight is from the camera.
     * 
     * @author dr
     */
    public double getDistance() {
        double deg = Constants.Limelight.ANGLE + this.getTy();
        double rad = deg * (Math.PI / 180.0);

        return ((Constants.Limelight.HUB_HEIGHT
                - Constants.Limelight.LL_HEIGHT)
                / Math.tan(rad)) / 12;
    }

    /**
     * Calculates the shooter speed based on the distance we got and a linear
     * interpolation equation (aka a regression).
     * 
     * @return the calculated shooter speed, in RPMs
     */
    public double calcShooterSpeed() {
        double distance = this.getDistance();
        double calcSpeed = 5017 + (-735 * distance) + (76.8 * Math.pow(distance, 2)) + (-2.4 * Math.pow(distance, 3));
        return calcSpeed;
    }

    /**
     * Sets the ledMode on the Limelight.
     * 
     * @param value the value to set the ledMode to (true is on, false is off).
     */
    public void enableLight(boolean value) {
        // https://docs.limelightvision.io/en/latest/getting_started.html#basic-programming
        table.getEntry("ledMode").setNumber(value ? 3 : 1);
    }

    /**
     * Sets the camMode on the Limelight. Setting this to false will increase the
     * exposure for use as a driver camera.
     * 
     * @param value the value to set the camMode to (true for vision, false for
     *              driver camera)
     */
    public void enableVision(boolean value) {
        // https://docs.limelightvision.io/en/latest/getting_started.html#basic-programming
        table.getEntry("camMode").setNumber(value ? 0 : 1);
    }

    /**
     * Sets the limelight into shoot mode (enabling camera and vision processing)
     */
    public void setShootingMode() {
        enableLight(true);
        enableVision(true);
    }

    /**
     * Sets the limelight into driver assist mode (disabling camera and vision
     * processing)
     */
    public void setDriverAssistMode() {
        enableLight(false);
        enableVision(false);
    }

    /**
     * Check if the limelight has a target.
     * 
     * @return true if it has a target, false if not
     */
    public boolean hasTarget() {
        return getTv() == 1.0;
    }
}
