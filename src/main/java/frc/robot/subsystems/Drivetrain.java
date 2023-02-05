package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.techhounds.houndutil.houndauto.AutoManager;
import com.techhounds.houndutil.houndlib.AprilTagPhotonCamera;
import com.techhounds.houndutil.houndlog.LogGroup;
import com.techhounds.houndutil.houndlog.LogProfileBuilder;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.loggers.DeviceLogger;
import com.techhounds.houndutil.houndlog.loggers.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FieldConstants;
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

    /** The PhotonVision cameras, used to detect the AprilTags. */
    private AprilTagPhotonCamera[] photonCameras = new AprilTagPhotonCamera[] {
            new AprilTagPhotonCamera(Constants.Vision.CAMERA_NAMES[0],
                    Constants.Vision.ROBOT_TO_CAMS[0]),
            new AprilTagPhotonCamera(Constants.Vision.CAMERA_NAMES[1],
                    Constants.Vision.ROBOT_TO_CAMS[1]),
            new AprilTagPhotonCamera(Constants.Vision.CAMERA_NAMES[2],
                    Constants.Vision.ROBOT_TO_CAMS[2]),
            new AprilTagPhotonCamera(Constants.Vision.CAMERA_NAMES[3],
                    Constants.Vision.ROBOT_TO_CAMS[3])
    };

    /**
     * The pose estimator, which takes in wheel odometry and latency-compensated
     * AprilTag measurements to provide an absolute position on the field.
     */
    private SwerveDrivePoseEstimator poseEstimator;

    /** An enum describing the two types of drive modes. */
    public enum DriveMode {
        ROBOT_RELATIVE,
        FIELD_ORIENTED
    }

    /** The mode of driving, either robot relative or field relative. */
    private DriveMode driveMode = DriveMode.FIELD_ORIENTED;

    /**
     * The internal register for the automatic 90° turns, so that the driver is able
     * to press the button multiple times.
     */
    private double turnRegister = 0;
    /** The initial position of the gyro, for the automatic 90° turns. */
    private double startingGyroAngle = 0;

    /** Whether to override the inputs of the driver for the automatic 90° turns. */
    private boolean isTurningEnabled = false;

    /**
     * The motion profiled controller that provides an angular velocity to complete
     * an automatic 90° turn.
     */
    private ProfiledPIDController turnController = new ProfiledPIDController(Constants.Drivetrain.Gains.TurnToAngle.kP,
            Constants.Drivetrain.Gains.TurnToAngle.kI,
            Constants.Drivetrain.Gains.TurnToAngle.kD,
            new TrapezoidProfile.Constraints(Constants.Auton.MAX_ANGULAR_VELOCITY,
                    Constants.Auton.MAX_ANGULAR_ACCELERATION));

    /** Initializes the drivetrain. */
    public Drivetrain() {
        zeroGyro();
        poseEstimator = new SwerveDrivePoseEstimator(
                Constants.Drivetrain.Geometry.KINEMATICS,
                getGyroRotation2d(),
                getSwerveModulePositions(),
                new Pose2d(new Translation2d(FieldConstants.LENGTH_METERS / 2.0, FieldConstants.WIDTH_METERS / 2.0),
                        new Rotation2d())); // center of the field

        turnController.setTolerance(0.05);

        LoggingManager.getInstance().addGroup("Drivetrain", new LogGroup(
                new Logger[] {
                        new DeviceLogger<Pigeon2>(pigeon, "Pigeon 2",
                                LogProfileBuilder.buildPigeon2LogItems(pigeon)),
                }));

        AutoManager.getInstance().setResetOdometryConsumer(this::resetPoseEstimator);

    }

    /**
     * Runs every 20ms. Do not run anything but odometry updating and debug code
     * here.
     */
    @Override
    public void periodic() {
        updatePoseEstimator();
    }

    /**
     * Returns the current drive mode of the swerve (robot-relative or
     * field-oriented).
     * 
     * @return the current drive mode
     */
    public DriveMode getDriveMode() {
        return driveMode;
    }

    /**
     * Sets the current drive mode of the swerve (robot-relative or field-oriented).
     * 
     * @param driveMode the drive mode to set
     */
    public void setDriveMode(DriveMode driveMode) {
        this.driveMode = driveMode;
    }

    /**
     * Get the profiled turning PID controller.
     * 
     * @return the profiled turning PID controller.
     */
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

    /**
     * Get the current swerve module positions (encoder positions).
     * 
     * @return the current swerve module positions
     */
    public SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
        };
    }

    /**
     * Get the current swerve module states (encoder velocities).
     * 
     * @return the current swerve module states
     */
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
     * @param states   an array of states (front left, front right, back left, back
     *                 right)
     * @param openLoop whether to set the velocities using a PID controller or open
     *                 loop
     * @param optimize whether to optimize the swerve module, making it not have to
     *                 rotate more than 90° at a time.
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
     * Gets the angle of the gyro represented by a Rotation2d.
     * 
     * @return the angle of the gyro represented by a Rotation2d
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
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Updates the pose estimator based on the swerve module positions and AprilTag
     * cameras.
     */
    private void updatePoseEstimator() {
        poseEstimator.update(getGyroRotation2d(), getSwerveModulePositions());

        for (int i = 0; i < photonCameras.length; i++) {
            Optional<EstimatedRobotPose> result = photonCameras[i]
                    .getEstimatedGlobalPose(poseEstimator.getEstimatedPosition());
            Field2d field = AutoManager.getInstance().getField();

            FieldObject2d fieldObject = field.getObject("apriltag_cam" + i + "_est_pose");
            if (result.isPresent()) {
                EstimatedRobotPose camPose = result.get();
                poseEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(),
                        camPose.timestampSeconds);
                fieldObject.setPose(camPose.estimatedPose.toPose2d());
            } else {
                // move it way off the screen to make it disappear
                fieldObject.setPose(new Pose2d(-100, -100, new Rotation2d()));
            }

            drawRobotOnField(AutoManager.getInstance().getField());
        }
    }

    /**
     * Resets the pose estimator to a specific pose.
     * 
     * @param pose the pose to reset to
     */
    public void resetPoseEstimator(Pose2d pose) {
        poseEstimator.resetPosition(getGyroRotation2d(), getSwerveModulePositions(), pose);
    }

    /**
     * Draws the robot on a Field2d. This will include the angles of the swerve
     * modules on the outsides of the robot box in Glass.
     * 
     * @param field the field to draw the robot on (usually
     *              {@code AutoManager.getInstance().getField()})
     */
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
     * Gets the output of the turn profiled PID controller,
     * 
     * @return the rotational velocity returned by the controller
     */
    public double getTurnControllerOutput() {
        return turnController.calculate(getGyroAngleRad());
    }

    /**
     * Checks if the turn controller should be enabled, and returns its current
     * state.
     * 
     * @return its current state
     */
    public boolean getTurnControllerEnabled() {
        if (isTurningEnabled) {
            if (turnController.atGoal()) {
                isTurningEnabled = false;
                turnRegister = 0;
            }
        }
        return isTurningEnabled;
    }

    /**
     * Creates a command that drives the robot based off of the X speed, Y speed,
     * and θ speed.
     * 
     * @param xSpeedSupplier     the supplier of the xSpeed in m/s
     * @param ySpeedSupplier     the supplier of the ySpeed in m/s
     * @param thetaSpeedSupplier the supplier of the thetaSpeed in θ/s
     * @param slowModeSupplier   the supplier of whether to run the bot in "slow
     *                           mode", where the inputs are multiplied by a lower
     *                           value.
     * @return the command
     */
    public CommandBase teleopDriveCommand(DoubleSupplier xSpeedSupplier, DoubleSupplier ySpeedSupplier,
            DoubleSupplier thetaSpeedSupplier, BooleanSupplier slowModeSupplier) {
        SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(Constants.Teleop.JOYSTICK_INPUT_RATE_LIMIT);
        SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(Constants.Teleop.JOYSTICK_INPUT_RATE_LIMIT);
        SlewRateLimiter thetaSpeedLimiter = new SlewRateLimiter(Constants.Teleop.JOYSTICK_INPUT_RATE_LIMIT);

        return new RunCommand(() -> {
            double xSpeed = xSpeedSupplier.getAsDouble();
            double ySpeed = ySpeedSupplier.getAsDouble();
            double thetaSpeed = thetaSpeedSupplier.getAsDouble();

            xSpeed = MathUtil.applyDeadband(xSpeed, 0.05);
            ySpeed = MathUtil.applyDeadband(ySpeed, 0.05);
            thetaSpeed = MathUtil.applyDeadband(thetaSpeed, 0.05);

            if (Constants.Teleop.IS_JOYSTICK_INPUT_RATE_LIMITED) {
                xSpeed = xSpeedLimiter.calculate(xSpeed);
                ySpeed = ySpeedLimiter.calculate(ySpeed);
                thetaSpeed = thetaSpeedLimiter.calculate(thetaSpeed);
            }

            if (!slowModeSupplier.getAsBoolean()) {
                xSpeed *= Constants.Teleop.PERCENT_LIMIT;
                ySpeed *= Constants.Teleop.PERCENT_LIMIT;
                thetaSpeed *= Constants.Teleop.PERCENT_LIMIT;
            } else {
                xSpeed *= Constants.Teleop.SLOW_MODE_PERCENT_LIMIT;
                ySpeed *= Constants.Teleop.SLOW_MODE_PERCENT_LIMIT;
                thetaSpeed *= Constants.Teleop.SLOW_MODE_PERCENT_LIMIT;
            }

            if (getTurnControllerEnabled()) {
                thetaSpeed = getTurnControllerOutput();
            }

            // the speeds are initially values from -1.0 to 1.0, so we multiply by the max
            // physical velocity to output in m/s.
            xSpeed *= Constants.Drivetrain.Geometry.MAX_PHYSICAL_VELOCITY_METERS_PER_SECOND;
            ySpeed *= Constants.Drivetrain.Geometry.MAX_PHYSICAL_VELOCITY_METERS_PER_SECOND;
            thetaSpeed *= Constants.Drivetrain.Geometry.MAX_PHYSICAL_VELOCITY_METERS_PER_SECOND;

            drive(
                    xSpeed, ySpeed, thetaSpeed,
                    getDriveMode());
        }, this);
    }

    /**
     * Creates a command that adds 90 degrees CCW to the internal register for the
     * turning PID controller.
     * 
     * Map this to a button input to turn 90 degrees CCW while still moving.
     * 
     * @return the command
     */
    public CommandBase turnWhileMovingCommand(boolean counterClockwise) {
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

    /**
     * Creates a command that sets the wheels to a "brake" configuration, where the
     * lateral side of each wheel is facing the center.
     * 
     * @return the command
     */
    public Command brakeCommand() {
        return new InstantCommand(() -> {
            setModuleStates(new SwerveModuleState[] {
                    new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(-45))
            });
        });
    }

    /**
     * Creates a command that sets the wheels to a "brake" configuration, where the
     * tread side of each wheel is facing the center.
     * 
     * @return the command
     */
    public Command brakeXCommand() {
        return new InstantCommand(() -> {
            setModuleStates(new SwerveModuleState[] {
                    new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(45))
            });
        });
    }

    /**
     * Creates a command that turns all of the wheels to a specified angle.
     * 
     * @return the command
     */
    public Command turnWheelsToAngleCommand(double angle) {
        return new InstantCommand(() -> {
            setModuleStates(new SwerveModuleState[] {
                    new SwerveModuleState(0, new Rotation2d(angle)),
                    new SwerveModuleState(0, new Rotation2d(angle)),
                    new SwerveModuleState(0, new Rotation2d(angle)),
                    new SwerveModuleState(0, new Rotation2d(angle))

            });
        });
    }

    /**
     * Creates a command that supplies SwerveModuleStates to follow a
     * PathPlannerTrajectory.
     * 
     * @return the command
     */
    public CommandBase pathFollowingCommand(PathPlannerTrajectory path) {
        return new PPSwerveControllerCommand(
                path,
                this::getPose,
                Constants.Drivetrain.Geometry.KINEMATICS,
                new PIDController(Constants.Drivetrain.Gains.Trajectories.xkP, 0, 0),
                new PIDController(Constants.Drivetrain.Gains.Trajectories.ykP, 0, 0),
                new PIDController(Constants.Drivetrain.Gains.Trajectories.thetakP, 0, 0),
                (s) -> this.setModuleStates(s, true, true),
                this);
    }

    /**
     * Creates a command that supplies SwerveModuleStates to follow a
     * PathPlannerTrajectory, but the holonomic rotation is overridden to always be
     * π rad/s.
     * 
     * @return the command
     */
    public CommandBase spinningPathFollowingCommand(PathPlannerTrajectory path) {
        return new PPSwerveControllerCommand(
                path,
                this::getPose,
                Constants.Drivetrain.Geometry.KINEMATICS,
                new PIDController(Constants.Drivetrain.Gains.Trajectories.xkP, 0, 0),
                new PIDController(Constants.Drivetrain.Gains.Trajectories.ykP, 0, 0),
                new PIDController(0, 0, 0),
                (s) -> {
                    ChassisSpeeds c = Constants.Drivetrain.Geometry.KINEMATICS.toChassisSpeeds(s);
                    c.omegaRadiansPerSecond = Math.PI;

                    this.setModuleStates(Constants.Drivetrain.Geometry.KINEMATICS.toSwerveModuleStates(c), true,
                            true);
                },
                this);
    }
}
