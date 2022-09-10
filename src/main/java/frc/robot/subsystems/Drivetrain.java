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
import frc.houndutil.houndlog.LogGroup;
import frc.houndutil.houndlog.LogProfileBuilder;
import frc.houndutil.houndlog.LoggingManager;
import frc.houndutil.houndlog.loggers.DeviceLogger;
import frc.houndutil.houndlog.loggers.Logger;
import frc.houndutil.houndlog.loggers.SendableLogger;
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
            Constants.Drivetrain.CANIDs.FrontLeft.TURN_MOTOR, Constants.Drivetrain.CANIDs.FrontLeft.TURN_ENCODER,
            false, false, false);

    /** The front right swerve module when looking at the bot from behind. */
    private SwerveModule frontRight = new SwerveModule("Drivetrain/Front Right Module",
            Constants.Drivetrain.CANIDs.FrontRight.DRIVE_MOTOR,
            Constants.Drivetrain.CANIDs.FrontRight.TURN_MOTOR, Constants.Drivetrain.CANIDs.FrontRight.TURN_ENCODER,
            false, false, false);

    /** The back left swerve module when looking at the bot from behind. */
    private SwerveModule backLeft = new SwerveModule("Drivetrain/Back Left Module",
            Constants.Drivetrain.CANIDs.BackLeft.DRIVE_MOTOR,
            Constants.Drivetrain.CANIDs.BackLeft.TURN_MOTOR, Constants.Drivetrain.CANIDs.BackLeft.TURN_ENCODER,
            false, false, false);

    /** The back right swerve module when looking at the bot from behind. */
    private SwerveModule backRight = new SwerveModule("Drivetrain/Back Right Module",
            Constants.Drivetrain.CANIDs.BackRight.DRIVE_MOTOR,
            Constants.Drivetrain.CANIDs.BackRight.TURN_MOTOR, Constants.Drivetrain.CANIDs.BackRight.TURN_ENCODER,
            false, false, false);

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
    private DriveMode driveMode = DriveMode.ROBOT_RELATIVE;

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
        odometry.update(new Rotation2d(navx.getYaw()), frontLeft.getState(), frontRight.getState(), backLeft.getState(),
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
     * @param xSpeed        the speed in the x direction
     * @param ySpeed        the speed in the y direction
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
                chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, thetaSpeed, navx.getRotation2d());
                break;
        }
        SwerveModuleState[] states = Constants.Drivetrain.Geometry.KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(states);
    }

    /**
     * Desaturates the wheel speeds and sets the states of the swerve modules.
     * 
     * @param states an array of states (front left, front right, back left, back
     *               right)
     */
    public void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states,
                Constants.Drivetrain.Geometry.MAX_PHYSICAL_VELOCITY_METERS_PER_SECOND);
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

    /**
     * Gets the average position of the encoders on the drive motors. This is only
     * useful when driving straight.
     * 
     * @return the average position of the encoders on the drive motors
     */
    public double getDriveEncoderPosition() {
        return (frontLeft.getDriveEncoderPosition() + frontRight.getDriveEncoderPosition()
                + backLeft.getDriveEncoderPosition() + backRight.getDriveEncoderPosition()) / 4.0;
    }

    /**
     * Resets the positions of the encoders on the drive motors.
     */
    public void resetDriveEncoders() {
        frontLeft.resetDriveEncoder();
        frontRight.resetDriveEncoder();
        backLeft.resetDriveEncoder();
        backRight.resetDriveEncoder();
    }
}
