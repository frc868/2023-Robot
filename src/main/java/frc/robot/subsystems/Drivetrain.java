package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.unmanaged.Unmanaged;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.techhounds.houndutil.houndauto.AutoManager;
import com.techhounds.houndutil.houndlib.AdvantageScopeSerializer;
import com.techhounds.houndutil.houndlib.Rectangle2d;
import com.techhounds.houndutil.houndlib.commands.DeferredCommand;
import com.techhounds.houndutil.houndlib.swerve.NEOCoaxialSwerveModule;
import com.techhounds.houndutil.houndlog.interfaces.Log;
import com.techhounds.houndutil.houndlog.interfaces.LoggedObject;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.GamePieceLocation;
import frc.robot.GridInterface;
import frc.robot.Modes;
import frc.robot.Modes.RobotState;

/**
 * The drivetrain subsystem, containing four swerve modules and odometry-related
 * information.
 * 
 * @author dr
 */
@LoggedObject
public class Drivetrain extends SubsystemBase {
    /** The front left swerve module when looking at the bot from behind. */
    @Log(name = "Front Left Module", groups = "Modules")
    private NEOCoaxialSwerveModule frontLeft = new NEOCoaxialSwerveModule(
            Constants.CAN.FRONT_LEFT_DRIVE_MOTOR_ID,
            Constants.CAN.FRONT_LEFT_TURN_MOTOR_ID,
            Constants.CAN.FRONT_LEFT_TURN_ENCODER_ID,
            false, true, false,
            Constants.Geometries.Drivetrain.Offsets.FRONT_LEFT,
            Constants.Geometries.Drivetrain.SWERVE_CONSTANTS);

    /** The front right swerve module when looking at the bot from behind. */
    @Log(name = "Front Right Module", groups = "Modules")
    private NEOCoaxialSwerveModule frontRight = new NEOCoaxialSwerveModule(
            Constants.CAN.FRONT_RIGHT_DRIVE_MOTOR_ID,
            Constants.CAN.FRONT_RIGHT_TURN_MOTOR_ID,
            Constants.CAN.FRONT_RIGHT_TURN_ENCODER_ID,
            false, true, false,
            Constants.Geometries.Drivetrain.Offsets.FRONT_RIGHT,
            Constants.Geometries.Drivetrain.SWERVE_CONSTANTS);

    /** The back left swerve module when looking at the bot from behind. */
    @Log(name = "Back Left Module", groups = "Modules")
    private NEOCoaxialSwerveModule backLeft = new NEOCoaxialSwerveModule(
            Constants.CAN.BACK_LEFT_DRIVE_MOTOR_ID,
            Constants.CAN.BACK_LEFT_TURN_MOTOR_ID,
            Constants.CAN.BACK_LEFT_TURN_ENCODER_ID,
            false, true, false,
            Constants.Geometries.Drivetrain.Offsets.BACK_LEFT,
            Constants.Geometries.Drivetrain.SWERVE_CONSTANTS);

    /** The back right swerve module when looking at the bot from behind. */
    @Log(name = "Back Right Module", groups = "Modules")
    private NEOCoaxialSwerveModule backRight = new NEOCoaxialSwerveModule(
            Constants.CAN.BACK_RIGHT_DRIVE_MOTOR_ID,
            Constants.CAN.BACK_RIGHT_TURN_MOTOR_ID,
            Constants.CAN.BACK_RIGHT_TURN_ENCODER_ID,
            false, true, false,
            Constants.Geometries.Drivetrain.Offsets.BACK_RIGHT,
            Constants.Geometries.Drivetrain.SWERVE_CONSTANTS);

    /** The gyroscope on the robot. */
    @Log(name = "Pigeon 2")
    private Pigeon2 pigeon = new Pigeon2(0);

    /**
     * The pose estimator, which takes in wheel odometry and latency-compensated
     * AprilTag measurements to provide an absolute position on the field.
     */
    @Log(name = "Pose Estimator")
    private SwerveDrivePoseEstimator poseEstimator;

    /**
     * The yaw of the gyro in simulations. This value is not used when the robot is
     * running IRL.
     */
    private double simYaw;

    /**
     * Whether to override the inputs of the driver for maintaining or turning to a
     * specific angle.
     */
    private boolean isControlledRotationEnabled = false;

    /**
     * The controller that allows the drivetrain to maintain or turn to a specific
     * angle
     */
    @Log(name = "Rotation Controller")
    private ProfiledPIDController rotationController = new ProfiledPIDController(Constants.Gains.TurnToAngle.kP,
            Constants.Gains.TurnToAngle.kI,
            Constants.Gains.TurnToAngle.kD,
            new TrapezoidProfile.Constraints(20 * Math.PI,
                    20 * Math.PI));

    /** An enum describing the two types of drive modes. */
    public enum DriveMode {
        ROBOT_RELATIVE,
        FIELD_ORIENTED
    }

    /** The mode of driving, either robot relative or field relative */
    private DriveMode driveMode = DriveMode.FIELD_ORIENTED;

    @Log(name = "commandedXVelocity", groups = "Control")
    private double commandedX = 0.0;

    @Log(name = "Commanded Y Velocity", groups = "Control")
    private double commandedY = 0.0;

    @Log(name = "Commanded Theta Velocity", groups = "Control")
    private double commandedTheta = 0.0;

    private SwerveModuleState[] commandedStates = new SwerveModuleState[] { new SwerveModuleState(),
            new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState() };
    @Log(name = "Commanded States", groups = "Control")
    Supplier<double[]> commandedStatesSupplier = () -> AdvantageScopeSerializer
            .serializeSwerveModuleStates(commandedStates);

    @Log(name = "Measured States", groups = "Control")
    Supplier<double[]> measuredStatesSupplier = () -> AdvantageScopeSerializer
            .serializeSwerveModuleStates(getSwerveModuleStates());

    @Log(name = "Measured X Position", groups = "Control")
    Supplier<Double> xPositionSupplier = () -> getPose().getX();
    @Log(name = "Measured Y Position", groups = "Control")
    Supplier<Double> yPositionSupplier = () -> getPose().getY();
    @Log(name = "Measured Theta Rotation", groups = "Control")
    Supplier<Double> thetaRotationSupplier = () -> getPose().getRotation().getRadians();

    @Log(name = "Measured X Velocity", groups = "Control")
    Supplier<Double> xVelocitySupplier = () -> Constants.Geometries.Drivetrain.KINEMATICS
            .toChassisSpeeds(getSwerveModuleStates()).vxMetersPerSecond;
    @Log(name = "Measured Y Velocity", groups = "Control")
    Supplier<Double> yVelocitySupplier = () -> Constants.Geometries.Drivetrain.KINEMATICS
            .toChassisSpeeds(getSwerveModuleStates()).vyMetersPerSecond;
    @Log(name = "Measured Theta Velocity", groups = "Control")
    Supplier<Double> thetaVelocitySupplier = () -> Constants.Geometries.Drivetrain.KINEMATICS
            .toChassisSpeeds(getSwerveModuleStates()).omegaRadiansPerSecond;

    /** Initializes the drivetrain. */
    public Drivetrain() {
        zeroGyro();
        poseEstimator = new SwerveDrivePoseEstimator(
                Constants.Geometries.Drivetrain.KINEMATICS,
                getGyroRotation2d(),
                getSwerveModulePositions(),
                new Pose2d(1.84, 5.06, new Rotation2d()));

        rotationController.setTolerance(0.05);
        rotationController.enableContinuousInput(0, 2 * Math.PI);

        AutoManager.getInstance().setResetOdometryConsumer(this::resetPoseEstimator);

        if (RobotBase.isSimulation()) {
            simYaw = 0;
        }
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
     * Updates simulation-specific variables.
     */
    @Override
    public void simulationPeriodic() {
        ChassisSpeeds chassisSpeed = Constants.Geometries.Drivetrain.KINEMATICS
                .toChassisSpeeds(getSwerveModuleStates());
        simYaw += chassisSpeed.omegaRadiansPerSecond * 0.02;

        Unmanaged.feedEnable(20);
        pigeon.getSimCollection().setRawHeading(Units.radiansToDegrees(simYaw));
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
     * Creates a command that zeros the gyro in whichever direction the bot is
     * pointing. Only use this if
     * the bot is straight, otherwise it will throw off field-oriented control.
     * 
     * @return the command
     */
    public CommandBase zeroGyroCommand() {
        return runOnce(() -> zeroGyro());
    }

    /**
     * Gets the current pose of the drivetrain.
     * 
     * @return the current pose of the drivetrain
     */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public SwerveDrivePoseEstimator getPoseEstimator() {
        return poseEstimator;
    }

    /**
     * Updates the pose estimator based on the swerve module positions and AprilTag
     * cameras.
     */
    private void updatePoseEstimator() {
        poseEstimator.update(getGyroRotation2d(), getSwerveModulePositions());
        drawRobotOnField(AutoManager.getInstance().getField());
    }

    /**
     * Resets the pose estimator to a specific pose.
     * 
     * @param pose the pose to reset to
     */
    public void resetPoseEstimator(Pose2d pose) {
        poseEstimator.resetPosition(getGyroRotation2d(), getSwerveModulePositions(), pose);
        pigeon.setYaw(pose.getRotation().getDegrees());
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
                        new Transform2d(Constants.Geometries.Drivetrain.SWERVE_MODULE_LOCATIONS[0],
                                getSwerveModuleStates()[0].angle)));
        field.getObject("frontRight").setPose(
                getPose().transformBy(
                        new Transform2d(Constants.Geometries.Drivetrain.SWERVE_MODULE_LOCATIONS[1],
                                getSwerveModuleStates()[1].angle)));
        field.getObject("backLeft").setPose(
                getPose().transformBy(
                        new Transform2d(Constants.Geometries.Drivetrain.SWERVE_MODULE_LOCATIONS[2],
                                getSwerveModuleStates()[2].angle)));
        field.getObject("backRight").setPose(
                getPose().transformBy(
                        new Transform2d(Constants.Geometries.Drivetrain.SWERVE_MODULE_LOCATIONS[3],
                                getSwerveModuleStates()[3].angle)));
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
        if (DriverStation.getAlliance() == Alliance.Red && driveMode == DriveMode.FIELD_ORIENTED) {
            xSpeed *= -1;
            ySpeed *= -1;
        }

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

        commandedX = chassisSpeeds.vxMetersPerSecond;
        commandedY = chassisSpeeds.vyMetersPerSecond;
        commandedTheta = chassisSpeeds.omegaRadiansPerSecond;

        SwerveModuleState[] states = Constants.Geometries.Drivetrain.KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states,
                Constants.Geometries.Drivetrain.MAX_DRIVING_VELOCITY_METERS_PER_SECOND);
        commandedStates = states;
        setModuleStates(states, true, true);
    }

    /**
     * Stops the drivetrain.
     */
    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
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
            DoubleSupplier thetaSpeedSupplier, BooleanSupplier isInputCubed) {
        SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(Constants.Teleop.JOYSTICK_INPUT_RATE_LIMIT);
        SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(Constants.Teleop.JOYSTICK_INPUT_RATE_LIMIT);
        SlewRateLimiter thetaSpeedLimiter = new SlewRateLimiter(Constants.Teleop.JOYSTICK_INPUT_RATE_LIMIT);

        return run(() -> {
            double xSpeed = xSpeedSupplier.getAsDouble();
            double ySpeed = ySpeedSupplier.getAsDouble();
            double thetaSpeed = thetaSpeedSupplier.getAsDouble();

            xSpeed = MathUtil.applyDeadband(xSpeed, 0.05);
            ySpeed = MathUtil.applyDeadband(ySpeed, 0.05);
            thetaSpeed = MathUtil.applyDeadband(thetaSpeed, 0.05);

            if (isInputCubed.getAsBoolean()) {
                xSpeed = Math.pow(xSpeed, 3);
                ySpeed = Math.pow(ySpeed, 3);
                thetaSpeed = Math.pow(thetaSpeed, 3);
            }

            if (Constants.Teleop.IS_JOYSTICK_INPUT_RATE_LIMITED) {
                xSpeed = xSpeedLimiter.calculate(xSpeed);
                ySpeed = ySpeedLimiter.calculate(ySpeed);
                thetaSpeed = thetaSpeedLimiter.calculate(thetaSpeed);
            }

            if (isControlledRotationEnabled) {
                thetaSpeed = rotationController.calculate(getGyroRotation2d().getRadians());
            }

            // the speeds are initially values from -1.0 to 1.0, so we multiply by the max
            // physical velocity to output in m/s.
            xSpeed *= Constants.Geometries.Drivetrain.MAX_DRIVING_VELOCITY_METERS_PER_SECOND;
            ySpeed *= Constants.Geometries.Drivetrain.MAX_DRIVING_VELOCITY_METERS_PER_SECOND;
            thetaSpeed *= Constants.Geometries.Drivetrain.MAX_DRIVING_VELOCITY_METERS_PER_SECOND;

            drive(
                    xSpeed, ySpeed, thetaSpeed,
                    getDriveMode());
        }).withName("Teleop Drive Command");
    }

    /**
     * Creates a command that allows for rotation to any angle.
     * 
     * Map this to a button input to rotate while still translating.
     * 
     * @return the command
     */
    public CommandBase controlledRotateCommand(double angle, boolean fieldRelative) {
        return Commands.startEnd(() -> {
            if (!isControlledRotationEnabled) {
                rotationController.reset(getGyroRotation2d().getRadians());
            }
            isControlledRotationEnabled = true;
            if (fieldRelative && DriverStation.getAlliance() == Alliance.Red)
                rotationController.setGoal(angle + Math.PI);
            else
                rotationController.setGoal(angle);
        }, () -> {
            isControlledRotationEnabled = false;
        }).withName("Turn While Moving");
    }

    /**
     * Creates a command that sets the current drive mode of the swerve
     * (robot-relative or field-oriented).
     * 
     * @param driveMode the drive mode to set
     */
    public CommandBase setDriveModeCommand(DriveMode driveMode) {
        return runOnce(() -> this.driveMode = driveMode);
    }

    /**
     * Creates a command that sets the wheels to a "brake" configuration, where the
     * lateral side of each wheel is facing the center.
     * 
     * @return the command
     */
    public CommandBase brakeOCommand() {
        return Commands.runOnce(() -> {
            setModuleStates(new SwerveModuleState[] {
                    new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(-45))
            });
        }).withName("Brake");
    }

    /**
     * Creates a command that sets the wheels to a "brake" configuration, where the
     * tread side of each wheel is facing the center.
     * 
     * @return the command
     */
    public CommandBase brakeXCommand() {
        return Commands.runOnce(() -> {
            setModuleStates(new SwerveModuleState[] {
                    new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(45))
            });
        }).withName("Brake X");
    }

    /**
     * Creates a command that turns all of the wheels to a specified angle.
     * 
     * @return the command
     */
    public CommandBase turnWheelsToAngleCommand(double angle) {
        return Commands.runOnce(() -> {
            setModuleStates(new SwerveModuleState[] {
                    new SwerveModuleState(0, new Rotation2d(angle)),
                    new SwerveModuleState(0, new Rotation2d(angle)),
                    new SwerveModuleState(0, new Rotation2d(angle)),
                    new SwerveModuleState(0, new Rotation2d(angle))

            });
        }).withName("Turn Wheels To Angle");
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
                Constants.Geometries.Drivetrain.KINEMATICS,
                new PIDController(Constants.Gains.Trajectories.xkP, 0, 0),
                new PIDController(Constants.Gains.Trajectories.ykP, 0, 0),
                new PIDController(Constants.Gains.Trajectories.thetakP, 0, 0),
                (s) -> this.setModuleStates(s, true, true), // TODO: change to closed loop
                this);
    }

    /**
     * Creates a command that supplies SwerveModuleStates to follow a
     * PathPlannerTrajectory.
     * 
     * @return the command
     */
    public CommandBase pathFollowingWithEventsCommand(PathPlannerTrajectory path) {
        return new FollowPathWithEvents(pathFollowingCommand(path), path.getMarkers(),
                AutoManager.getInstance().getEventMap());
    }

    /**
     * Creates a command that supplies SwerveModuleStates to follow a
     * PathPlannerTrajectory.
     * 
     * @return the command
     */
    public CommandBase moveDeltaPathFollowingCommand(Transform2d delta, PathConstraints constraints) {
        return new DeferredCommand(() -> pathFollowingCommand(PathPlanner.generatePath(
                constraints,
                new PathPoint(
                        getPose().getTranslation(),
                        Rotation2d.fromDegrees(180),
                        getPose().getRotation()),
                new PathPoint(
                        getPose().plus(delta).getTranslation(),
                        Rotation2d.fromDegrees(180),
                        getPose().getRotation()))));
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
                Constants.Geometries.Drivetrain.KINEMATICS,
                new PIDController(Constants.Gains.Trajectories.xkP, 0, 0),
                new PIDController(Constants.Gains.Trajectories.ykP, 0, 0),
                new PIDController(0, 0, 0),
                (s) -> {
                    ChassisSpeeds c = Constants.Geometries.Drivetrain.KINEMATICS.toChassisSpeeds(s);
                    c.omegaRadiansPerSecond = Math.PI;

                    this.setModuleStates(Constants.Geometries.Drivetrain.KINEMATICS.toSwerveModuleStates(c), true,
                            true);
                },
                this);
    }

    // AUTODRIVE

    /**
     * Positive distances are towards the grid.
     * 
     * @param distance
     * @param drivetrain
     * @return
     */
    public CommandBase driveDistanceDeltaCommand(double distance,
            PathConstraints constraints) {
        return moveDeltaPathFollowingCommand(
                new Transform2d(
                        new Translation2d(distance, 0),
                        new Rotation2d(Math.PI)),
                constraints)
                .withTimeout(2).withName("Drive Distance Delta");
    }

    /**
     * Positive distances are towards the grid.
     * 
     * @param distance
     * @param drivetrain
     * @return
     */
    public CommandBase driveDeltaCommand(double distance) {
        return driveDistanceDeltaCommand(distance, new PathConstraints(3, 1));
    }

    /**
     * Creates a trajectory to the specified game piece location based on the
     * drivetrain's current position, and displays it on the AutoManager's Field2d.
     * 
     * @param drivetrain
     * @param gridInterface
     * @return the trajectory
     */
    public PathPlannerTrajectory getAutoDriveTraj(Supplier<RobotState> currentState,
            Supplier<GamePieceLocation> location) {
        return getAutoDriveTraj(new PathConstraints(3, 2), currentState, location);
    }

    public PathPlannerTrajectory getAutoDriveTraj(PathConstraints constraints, Supplier<RobotState> currentState,
            Supplier<GamePieceLocation> location) {

        Pose2d targetPose;
        Map<Rectangle2d, Pose2d[]> map;
        if (currentState.get() == RobotState.SEEKING) {
            targetPose = DriverStation.getAlliance() == Alliance.Blue
                    ? FieldConstants.Blue.Substations.SINGLE_SUBSTATION
                    : FieldConstants.Red.Substations.SINGLE_SUBSTATION;

            map = FieldConstants.AutoDrive.HP_STATION_ZONE_TO_INTERMEDIARY
                    .get(DriverStation.getAlliance());

        } else if (currentState.get() == RobotState.SCORING) {
            if (location.get() == null) {
                return new PathPlannerTrajectory();
            } // so that the rest doesn't throw an exception and crash code

            targetPose = FieldConstants.scoringLocationMap.get(DriverStation.getAlliance())
                    .get(location.get());

            map = FieldConstants.AutoDrive.SCORING_AREA_ZONE_TO_INTERMEDIARY
                    .get(DriverStation.getAlliance());

        } else {
            return new PathPlannerTrajectory();
        }

        Pose2d[] intermediaryPoses = null;
        for (Rectangle2d rect : map.keySet()) {
            if (rect.isInRect(getPose())) {
                intermediaryPoses = map.get(rect);
            }
        }

        ArrayList<PathPoint> pathPoints = new ArrayList<PathPoint>();

        // in order to angle towards target

        try {
            pathPoints.add(
                    new PathPoint(
                            getPose().getTranslation(),
                            new Rotation2d(intermediaryPoses[0].getX() - getPose().getX(),
                                    intermediaryPoses[0].getY() - getPose().getY()),
                            getPose().getRotation()));
            for (Pose2d pose : intermediaryPoses) {
                pathPoints.add(new PathPoint(pose.getTranslation(), pose.getRotation(), pose.getRotation()));
            }
        } catch (NullPointerException e) {
            pathPoints.add(
                    new PathPoint(
                            getPose().getTranslation(),
                            new Rotation2d(targetPose.getX() - getPose().getX(),
                                    targetPose.getY() - getPose().getY()),
                            getPose().getRotation()));
        }
        Pose2d last;
        try {
            last = intermediaryPoses[intermediaryPoses.length - 1];
        } catch (Exception e) {
            last = getPose();
        }
        pathPoints.add(new PathPoint(targetPose.getTranslation(),
                new Rotation2d(targetPose.getX() - last.getX(),
                        targetPose.getY() - last.getY()),
                targetPose.getRotation()));

        PathPlannerTrajectory traj = PathPlanner.generatePath(
                constraints,
                pathPoints);

        AutoManager.getInstance().getField().getObject("AutoDrive Trajectory").setTrajectory(traj);
        return traj;
    }

    /**
     * Creates a command that drives to the location specified by the operator
     * interface.
     * 
     * The command will not run if the location has not been set.
     * 
     * @param drivetrain
     * @param gridInterface
     * @return the command
     */
    public CommandBase autoDriveCommand(GridInterface gridInterface) {
        Supplier<Command> pathFollowingCommandSupplier = () -> pathFollowingCommand(
                getAutoDriveTraj(Modes::getRobotState,
                        () -> gridInterface.getSetLocation().orElseGet(null)));

        // this is a proxy command because we have to do things with the trajectory
        // every time before passing it into the `drivetrain.pathFollowingCommand`
        // method.
        return Commands.either(
                new DeferredCommand(pathFollowingCommandSupplier).finallyDo((d) -> {
                    stop();
                }),
                Modes.singularErrorCommand(() -> "Grid interface location not present"),
                () -> gridInterface.getSetLocation().isPresent() || Modes.getRobotState() == RobotState.SEEKING)
                .withName("Auto Drive");
    }

    /**
     * Creates a command that drives to the location specified by the operator
     * interface.
     * 
     * The command will not run if the location has not been set.
     * 
     * @param drivetrain
     * @param gridInterface
     * @return the command
     */
    public CommandBase autoDriveCommand(
            Supplier<RobotState> state,
            Supplier<GamePieceLocation> location) {
        return autoDriveCommand(new PathConstraints(3, 2), state, location);
    }

    public CommandBase autoDriveCommand(
            PathConstraints constraints,
            Supplier<RobotState> state,
            Supplier<GamePieceLocation> location) {
        Supplier<Command> pathFollowingCommandSupplier = () -> pathFollowingCommand(
                getAutoDriveTraj(constraints, state, location));

        // this is a proxy command because we have to do things with the trajectory
        // every time before passing it into the `drivetrain.pathFollowingCommand`
        // method.
        return new DeferredCommand(pathFollowingCommandSupplier, this)
                .finallyDo((d) -> stop())
                .withName("Auto Drive");
    }

    /**
     * Creates a command that supplies SwerveModuleStates to follow a
     * PathPlannerTrajectory, but the holonomic rotation is overridden to always be
     * π rad/s.
     * 
     * @return the command
     */
    public CommandBase chargeStationBalanceCommand() {
        PIDController controller = new PIDController(0.035, 0, 0.004);
        controller.setTolerance(3);
        return new PIDCommand(
                controller,
                () -> (-pigeon.getRoll()),
                0,
                (d) -> drive(
                        d > 0.6
                                ? (d > 0 ? 0.6 : -0.6)
                                : d,
                        0, 0, DriveMode.ROBOT_RELATIVE),
                this)
                .finallyDo((d) -> this.stop());
    }

    public CommandBase setDriveCurrentLimitCommand(int currentLimit) {
        return Commands.runOnce(() -> {
            frontLeft.setDriveCurrentLimit(currentLimit);
            frontRight.setDriveCurrentLimit(currentLimit);
            backLeft.setDriveCurrentLimit(currentLimit);
            backRight.setDriveCurrentLimit(currentLimit);
        });
    }

}
