package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.techhounds.houndutil.houndlog.LogGroup;
import com.techhounds.houndutil.houndlog.LogProfileBuilder;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.loggers.DeviceLogger;
import com.techhounds.houndutil.houndlog.loggers.Logger;
import com.techhounds.houndutil.houndlog.loggers.SendableLogger;
import frc.robot.Constants;
import frc.robot.utils.SwerveModule;

/**
 * The drivetrain subsystem, containing four swerve modules and odometry-related
 * information.
 * 
 * @author dr
 */
public class Drivetrain extends SubsystemBase {
    /** The front left swerve module when looking at the bot from behind. */
    private SwerveModule frontLeft = new SwerveModule("Drivetrain/Front Left Module",
            Constants.Drivetrain.CANIDs.FrontLeft.DRIVE_MOTOR,
            Constants.Drivetrain.CANIDs.FrontLeft.TURN_MOTOR,
            Constants.Drivetrain.CANIDs.FrontLeft.TURN_ENCODER,
            true, true, false,
            Constants.Drivetrain.Offsets.FRONT_LEFT);

    /** The front right swerve module when looking at the bot from behind. */
    private SwerveModule frontRight = new SwerveModule("Drivetrain/Front Right Module",
            Constants.Drivetrain.CANIDs.FrontRight.DRIVE_MOTOR,
            Constants.Drivetrain.CANIDs.FrontRight.TURN_MOTOR,
            Constants.Drivetrain.CANIDs.FrontRight.TURN_ENCODER,
            false, true, false,
            Constants.Drivetrain.Offsets.FRONT_RIGHT);

    /** The back left swerve module when looking at the bot from behind. */
    private SwerveModule backLeft = new SwerveModule("Drivetrain/Back Left Module",
            Constants.Drivetrain.CANIDs.BackLeft.DRIVE_MOTOR,
            Constants.Drivetrain.CANIDs.BackLeft.TURN_MOTOR,
            Constants.Drivetrain.CANIDs.BackLeft.TURN_ENCODER,
            false, true, false,
            Constants.Drivetrain.Offsets.BACK_LEFT);

    /** The back right swerve module when looking at the bot from behind. */
    private SwerveModule backRight = new SwerveModule("Drivetrain/Back Right Module",
            Constants.Drivetrain.CANIDs.BackRight.DRIVE_MOTOR,
            Constants.Drivetrain.CANIDs.BackRight.TURN_MOTOR,
            Constants.Drivetrain.CANIDs.BackRight.TURN_ENCODER,
            true, true, false,
            Constants.Drivetrain.Offsets.BACK_RIGHT);

    /** The NavX, connected via MXP to the RoboRIO. */
    private AHRS navx = new AHRS();

    /** Calculates odometry (robot's position) throughout the match. */
    private SwerveDriveOdometry odometry = new SwerveDriveOdometry(Constants.Drivetrain.Geometry.KINEMATICS,
            new Rotation2d(navx.getYaw()));

    /** Field that the robot's position can be drawn on and send via NT. */
    private Field2d field = new Field2d();

    /** An enum describing the two types of drive modes. */
    public enum DriveMode {
        ROBOT_RELATIVE,
        FIELD_ORIENTED
    }

    /** The mode of driving, either robot relative or field relative. */
    private DriveMode driveMode = DriveMode.FIELD_ORIENTED;

    /** Initializes the drivetrain. */
    public Drivetrain() {
        navx.reset();
        LoggingManager.getInstance().addGroup("Drivetrain", new LogGroup(
                new Logger[] {
                        new DeviceLogger<AHRS>(navx, "NavX",
                                LogProfileBuilder.buildNavXLogItems(navx)),
                        new SendableLogger("field", field),
                }));
    }

    /**
     * Runs every 20ms. Do not run anything but odometry updating and debug code
     * here.
     */
    @Override
    public void periodic() {
        odometry.update(new Rotation2d(
                navx.getYaw()),
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState());
        field.setRobotPose(odometry.getPoseMeters());
    }

    public DriveMode getDriveMode() {
        return driveMode;
    }

    public void setDriveMode(DriveMode driveMode) {
        this.driveMode = driveMode;
    }

    /**
     * Drives the drivetrain in a specified direction.
     * 
     * When driving the robot in a field-relative mode, positive x speeds correspond
     * to moving down the field.
     *
     * 
     * @param xSpeed        the speed in the x direction in m/s
     * @param ySpeed        the speed in the y direction in m/s
     * @param thetaSpeed    the rotational speed, in the counterclockwise direction,
     *                      and in rad/s (2pi is one rotation per second)
     * @param fieldRelative whether to control the robot relative to the field or to
     *                      the front of the bot
     */
    public void drive(double xSpeed, double ySpeed, double thetaSpeed, DriveMode driveMode) {
        ChassisSpeeds chassisSpeeds = null;
        switch (driveMode) {
            case ROBOT_RELATIVE:
                chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);
                break;
            case FIELD_ORIENTED:
                chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed,
                        thetaSpeed, navx.getRotation2d());
                break;
        }
        SwerveModuleState[] states = Constants.Drivetrain.Geometry.KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states,
                Constants.Drivetrain.Geometry.MAX_PHYSICAL_VELOCITY_METERS_PER_SECOND);
        setModuleStates(states);
    }

    /**
     * Sets the states of the swerve modules.
     * 
     * @param states an array of states (front left, front right, back left, back
     *               right)
     */
    public void setModuleStates(SwerveModuleState[] states) {
        frontLeft.setStateSimple(states[0]);
        frontRight.setStateSimple(states[1]);
        backLeft.setStateSimple(states[2]);
        backRight.setStateSimple(states[3]);
    }

    /**
     * Gets the angle of the gyro in degrees.
     * 
     * @return the angle of the gyro in degrees
     */
    public double getGyroAngle() {
        return navx.getRotation2d().getDegrees();
    }

    /**
     * Reset gyro angle.
     */
    public void resetGyroAngle() {
        navx.reset();
    }

    /**
     * Stops the drivetrain.
     */
    public void stop() {
        drive(0, 0, 0, DriveMode.ROBOT_RELATIVE);
    }

    /**
     * Gets the current pose of the drivetrain.
     * 
     * @return the current pose of the drivetrain
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    // public void drawRobotOnField(Field2d field) {
    // field.setRobotPose(getPose());
    // // Draw a pose that is based on the robot pose, but shifted by the
    // translation of the module relative to robot center,
    // // then rotated around its own center by the angle of the module.
    // field.getObject("frontLeft").setPose(
    // getPose().transformBy(new Transform2d(robotToModuleTL.get(FL),
    // getModuleStates()[0].angle))
    // );
    // field.getObject("frontRight").setPose(
    // getPose().transformBy(new
    // Transform2d(robotToModuleTL.get(FR),getModuleStates()[1].angle))
    // );
    // field.getObject("backLeft").setPose(
    // getPose().transformBy(new Transform2d(robotToModuleTL.get(BL),
    // getModuleStates()[2].angle))
    // );
    // field.getObject("backRight").setPose(
    // getPose().transformBy(new Transform2d(robotToModuleTL.get(BR),
    // getModuleStates()[3].angle))
    // );
    // }

}
