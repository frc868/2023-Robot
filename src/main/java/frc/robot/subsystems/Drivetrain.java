package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.techhounds.houndutil.houndlib.auto.AutoManager;
import com.techhounds.houndutil.houndlog.LogGroup;
import com.techhounds.houndutil.houndlog.LogProfileBuilder;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.loggers.DeviceLogger;
import com.techhounds.houndutil.houndlog.loggers.Logger;
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
            false, true, false,
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
            false, true, false,
            Constants.Drivetrain.Offsets.BACK_RIGHT);

    /** The Pigeon 2, the overpriced but really good gyro that we use. */
    private Pigeon2 pigeon = new Pigeon2(0);

    /** Calculates odometry (robot's position) throughout the match. */
    private SwerveDriveOdometry odometry;

    /** An enum describing the two types of drive modes. */
    public enum DriveMode {
        ROBOT_RELATIVE,
        FIELD_ORIENTED
    }

    /** The mode of driving, either robot relative or field relative. */
    private DriveMode driveMode = DriveMode.FIELD_ORIENTED;

    private double turnRegister = 0;
    private double startingGyroAngle = 0;
    private boolean isTurningEnabled = false;

    private ProfiledPIDController turnController = new ProfiledPIDController(Constants.Drivetrain.PID.TurnToAngle.kP,
            Constants.Drivetrain.PID.TurnToAngle.kI,
            Constants.Drivetrain.PID.TurnToAngle.kD,
            new TrapezoidProfile.Constraints(Constants.Auton.MAX_ANGULAR_VELOCITY,
                    Constants.Auton.MAX_ANGULAR_ACCELERATION));

    /** Initializes the drivetrain. */
    public Drivetrain() {
        zeroGyro();
        odometry = new SwerveDriveOdometry(Constants.Drivetrain.Geometry.KINEMATICS,
                getGyroRotation2d(), getSwerveModulePositions());

        turnController.setTolerance(0.05);

        LoggingManager.getInstance().addGroup("Drivetrain", new LogGroup(
                new Logger[] {
                        new DeviceLogger<Pigeon2>(pigeon, "Pigeon 2",
                                LogProfileBuilder.buildPigeon2LogItems(pigeon)),
                }));

        AutoManager.getInstance().setResetOdometryConsumer(this::resetOdometry);

    }

    /**
     * Runs every 20ms. Do not run anything but odometry updating and debug code
     * here.
     */
    @Override
    public void periodic() {
        odometry.update(getGyroRotation2d(), getSwerveModulePositions());
        drawRobotOnField(AutoManager.getInstance().getField());

        SmartDashboard.putNumber("starginGyro", startingGyroAngle);
        SmartDashboard.putNumber("turnRegister", turnRegister);

    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getGyroRotation2d(), getSwerveModulePositions(), pose);
    }

    public DriveMode getDriveMode() {
        return driveMode;
    }

    public void setDriveMode(DriveMode driveMode) {
        this.driveMode = driveMode;
    }

    public ProfiledPIDController getTurnController() {
        return turnController;
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
                        thetaSpeed, getGyroRotation2d());

                break;
        }
        SwerveModuleState[] states = Constants.Drivetrain.Geometry.KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states,
                Constants.Drivetrain.Geometry.MAX_PHYSICAL_VELOCITY_METERS_PER_SECOND);
        setModuleStates(states, true, true);
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
        };
    }

    public SwerveModuleState[] getSwerveModuleStates() {
        return new SwerveModuleState[] {
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState()
        };
    }

    /**
     * Sets the states of the swerve modules.
     * 
     * @param states an array of states (front left, front right, back left, back
     *               right)
     */
    public void setModuleStates(SwerveModuleState[] states) {
        frontLeft.setState(states[0]);
        frontRight.setState(states[1]);
        backLeft.setState(states[2]);
        backRight.setState(states[3]);
    }

    /**
     * Sets the states of the swerve modules.
     * 
     * @param states an array of states (front left, front right, back left, back
     *               right)
     */
    public void setModuleStates(SwerveModuleState[] states, boolean openLoop, boolean optimize) {
        frontLeft.setState(states[0], openLoop, optimize);
        frontRight.setState(states[1], openLoop, optimize);
        backLeft.setState(states[2], openLoop, optimize);
        backRight.setState(states[3], openLoop, optimize);
    }

    /**
     * Gets the angle (yaw) of the gyro in degrees.
     * 
     * @return the angle of the gyro in degrees
     */
    public double getGyroAngle() {
        return pigeon.getYaw();
    }

    /**
     * Gets the angle (yaw) of the gyro in radians.
     * 
     * @return the angle of the gyro in radians
     */
    public double getGyroAngleRad() {
        return Units.degreesToRadians(pigeon.getYaw());
    }

    /**
     * Gets the angle of the gyro in degrees.
     * 
     * @return the angle of the gyro in degrees
     */
    public Rotation2d getGyroRotation2d() {
        return Rotation2d.fromDegrees(pigeon.getYaw());
    }

    /**
     * Zeros the gyro in whichever direction the bot is pointing. Only use this if
     * the bot is straight, otherwise it will throw off field-oriented control.
     */
    public void zeroGyro() {
        pigeon.setYaw(0);
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

    public void drawRobotOnField(Field2d field) {
        field.setRobotPose(getPose());

        // Draw a pose that is based on the robot pose, but shifted by the
        // translation of the module relative to robot center,
        // then rotated around its own center by the angle of the module.
        field.getObject("frontLeft").setPose(
                getPose().transformBy(
                        new Transform2d(Constants.Drivetrain.Geometry.SWERVE_MODULE_LOCATIONS[0],
                                getSwerveModuleStates()[0].angle)));
        field.getObject("frontRight").setPose(
                getPose().transformBy(
                        new Transform2d(Constants.Drivetrain.Geometry.SWERVE_MODULE_LOCATIONS[1],
                                getSwerveModuleStates()[1].angle)));
        field.getObject("backLeft").setPose(
                getPose().transformBy(
                        new Transform2d(Constants.Drivetrain.Geometry.SWERVE_MODULE_LOCATIONS[2],
                                getSwerveModuleStates()[2].angle)));
        field.getObject("backRight").setPose(
                getPose().transformBy(
                        new Transform2d(Constants.Drivetrain.Geometry.SWERVE_MODULE_LOCATIONS[3],
                                getSwerveModuleStates()[3].angle)));
    }

    /**
     * Adds 90 degrees CCW to the internal register for the turning PID controller.
     * Map this to a button input to turn 90 degrees CCW while still moving.
     */
    public Command turnWhileMovingCommand(boolean counterClockwise) {
        return new InstantCommand(() -> {
            if (!isTurningEnabled) {
                startingGyroAngle = getGyroAngleRad();
                turnController.reset(startingGyroAngle);
                turnRegister = 0;
            }
            isTurningEnabled = true;
            turnRegister += counterClockwise ? Math.PI / 2.0 : -Math.PI / 2.0;

            turnController.setGoal(startingGyroAngle + turnRegister);
        });

    }

    public double getTurnControllerOutput() {
        return turnController.calculate(getGyroAngleRad());
    }

    public boolean getTurnControllerEnabled() {
        if (isTurningEnabled) {
            if (turnController.atGoal()) {
                isTurningEnabled = false;
                turnRegister = 0;
            }
        }
        return isTurningEnabled;
    }

}
