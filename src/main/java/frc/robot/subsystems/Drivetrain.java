package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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

public class Drivetrain extends SubsystemBase {
    private SwerveModule frontLeft = new SwerveModule("Drivetrain/Front Left Module",
            Constants.Drivetrain.CANIDs.FrontLeft.DRIVE_MOTOR,
            Constants.Drivetrain.CANIDs.FrontLeft.TURN_MOTOR, Constants.Drivetrain.CANIDs.FrontLeft.TURN_ENCODER, false,
            false);
    private SwerveModule frontRight = new SwerveModule("Drivetrain/Front Right Module",
            Constants.Drivetrain.CANIDs.FrontRight.DRIVE_MOTOR,
            Constants.Drivetrain.CANIDs.FrontRight.TURN_MOTOR, Constants.Drivetrain.CANIDs.FrontRight.TURN_ENCODER,
            false, false);
    private SwerveModule backLeft = new SwerveModule("Drivetrain/Back Left Module",
            Constants.Drivetrain.CANIDs.BackLeft.DRIVE_MOTOR,
            Constants.Drivetrain.CANIDs.BackLeft.TURN_MOTOR, Constants.Drivetrain.CANIDs.BackLeft.TURN_ENCODER, false,
            false);
    private SwerveModule backRight = new SwerveModule("Drivetrain/Back Right Module",
            Constants.Drivetrain.CANIDs.BackRight.DRIVE_MOTOR,
            Constants.Drivetrain.CANIDs.BackRight.TURN_MOTOR, Constants.Drivetrain.CANIDs.BackRight.TURN_ENCODER, false,
            false);

    // points on a square (if chassis width is 1m, points are (0.5, 0.5), (0.5,
    // -0.5), (-0.5, 0.5), (-0.5, -0.5))
    private Translation2d frontLeftLocation = new Translation2d(Constants.Drivetrain.CHASSIS_WIDTH / 2,
            Constants.Drivetrain.CHASSIS_WIDTH / 2);
    private Translation2d frontRightLocation = new Translation2d(Constants.Drivetrain.CHASSIS_WIDTH / 2,
            -Constants.Drivetrain.CHASSIS_WIDTH / 2);
    private Translation2d backLeftLocation = new Translation2d(-Constants.Drivetrain.CHASSIS_WIDTH / 2,
            Constants.Drivetrain.CHASSIS_WIDTH / 2);
    private Translation2d backRightLocation = new Translation2d(-Constants.Drivetrain.CHASSIS_WIDTH / 2,
            -Constants.Drivetrain.CHASSIS_WIDTH / 2);

    private AHRS navx = new AHRS();

    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation,
            backLeftLocation, backRightLocation);
    private SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(navx.getYaw()));
    private Field2d field = new Field2d();

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

    /**
     * Sets the states of each swerve module such that the chassis drives in a
     * specified direction.
     * 
     * When driving the robot in a field-relative mode, positive x speeds correspond
     * to moving down the field.
     *
     * 
     * @param xSpeed        The speed in the x direction.
     * @param ySpeed        The speed in the y direction.
     * @param thetaSpeed    The rotational speed, in the counterclockwise direction,
     *                      and in rad/s (2pi is one rotation per second).
     * @param fieldRelative Whether to control the robot relative to the field or to
     *                      the front of the bot.
     */
    public void drive(double xSpeed, double ySpeed, double thetaSpeed, boolean fieldRelative) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, thetaSpeed, navx.getRotation2d())
                        : new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed));
        setModuleStates(states);
    }

    public void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Drivetrain.MAX_VELOCITY);
        frontLeft.setState(states[0]);
        frontRight.setState(states[1]);
        backLeft.setState(states[2]);
        backRight.setState(states[3]);
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public double getGyroAngle() {
        return navx.getRotation2d().getDegrees();
    }

    public void stop() {
        drive(0, 0, 0, false);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public double getDrivePosition() {
        return (frontLeft.getDriveEncoderPosition() + frontRight.getDriveEncoderPosition()
                + backLeft.getDriveEncoderPosition() + backRight.getDriveEncoderPosition()) / 4.0;
    }

    public void resetDriveEncoders() {
        frontLeft.resetDriveEncoder();
        frontRight.resetDriveEncoder();
        backLeft.resetDriveEncoder();
        backRight.resetDriveEncoder();
    }

}
