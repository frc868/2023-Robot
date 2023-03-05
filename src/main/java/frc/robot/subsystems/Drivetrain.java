package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.unmanaged.Unmanaged;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.revrobotics.CANSparkMax.IdleMode;
import com.techhounds.houndutil.houndauto.AutoManager;
import com.techhounds.houndutil.houndlog.LogGroup;
import com.techhounds.houndutil.houndlog.LogProfileBuilder;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.enums.LogLevel;
import com.techhounds.houndutil.houndlog.loggers.DeviceLogger;
import com.techhounds.houndutil.houndlog.logitems.DoubleLogItem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
            Constants.CAN.FRONT_LEFT_DRIVE_MOTOR,
            Constants.CAN.FRONT_LEFT_TURN_MOTOR,
            Constants.CAN.FRONT_LEFT_TURN_ENCODER,
            false, true, false,
            Constants.Geometries.Drivetrain.Offsets.FRONT_LEFT);

    /** The front right swerve module when looking at the bot from behind. */
    private SwerveModule frontRight = new SwerveModule("Drivetrain/Front Right Module",
            Constants.CAN.FRONT_RIGHT_DRIVE_MOTOR,
            Constants.CAN.FRONT_RIGHT_TURN_MOTOR,
            Constants.CAN.FRONT_RIGHT_TURN_ENCODER,
            false, true, false,
            Constants.Geometries.Drivetrain.Offsets.FRONT_RIGHT);

    /** The back left swerve module when looking at the bot from behind. */
    private SwerveModule backLeft = new SwerveModule("Drivetrain/Back Left Module",
            Constants.CAN.BACK_LEFT_DRIVE_MOTOR,
            Constants.CAN.BACK_LEFT_TURN_MOTOR,
            Constants.CAN.BACK_LEFT_TURN_ENCODER,
            false, true, false,
            Constants.Geometries.Drivetrain.Offsets.BACK_LEFT);

    /** The back right swerve module when looking at the bot from behind. */
    private SwerveModule backRight = new SwerveModule("Drivetrain/Back Right Module",
            Constants.CAN.BACK_RIGHT_DRIVE_MOTOR,
            Constants.CAN.BACK_RIGHT_TURN_MOTOR,
            Constants.CAN.BACK_RIGHT_TURN_ENCODER,
            false, true, false,
            Constants.Geometries.Drivetrain.Offsets.BACK_RIGHT);

    /** The Pigeon 2, the overpriced but really good gyro that we use. */
    private Pigeon2 pigeon = new Pigeon2(0);

    /**
     * The pose estimator, which takes in wheel odometry and latency-compensated
     * AprilTag measurements to provide an absolute position on the field.
     */
    private SwerveDrivePoseEstimator poseEstimator;

    // /**
    // * The internal register for the automatic 90° turns, so that the driver is
    // able
    // * to press the button multiple times.
    // */
    // private double turnRegister = 0;
    // /** The initial position of the gyro, for the automatic 90° turns. */
    // private double startingGyroAngle = 0;

    /** Whether to override the inputs of the driver for the automatic 90° turns. */
    private boolean isTurningEnabled = false;

    /**
     * The yaw of the gyro in simulations. This value is not used when the robot is
     * running IRL.
     */
    private double simYaw;

    /**
     * The motion profiled controller that provides an angular velocity to complete
     * an automatic 90° turn.
     */
    private ProfiledPIDController turnController = new ProfiledPIDController(Constants.Gains.TurnToAngle.kP,
            Constants.Gains.TurnToAngle.kI,
            Constants.Gains.TurnToAngle.kD,
            new TrapezoidProfile.Constraints(10 * Math.PI,
                    10 * Math.PI));

    /** An enum describing the two types of drive modes. */
    public enum DriveMode {
        ROBOT_RELATIVE,
        FIELD_ORIENTED
    }

    /** The mode of driving, either robot relative or field relative. */
    private DriveMode driveMode = DriveMode.FIELD_ORIENTED;

    /**
     * An enum describing the speed of the drivetrain.
     */
    public enum SpeedMode {
        // these limits are the defaults, but they can be adjusted by the driver
        /**
         * The slow mode, where the robot runs at 25% speed and the controller is lit up
         * red.
         */
        SLOW(0.25),
        /**
         * The fast mode, where the robot runs at 65% speed and the controller is lit up
         * green.
         */
        FAST(0.65),
        /**
         * The ultra-fast mode, where the robot is not limited (be careful) and the
         * controller is lit up blue.
         */
        ULTRA(1.0);

        /** The percent limit for this speedMode */
        private double limit;

        /** Initializes the SpeedMode. */
        private SpeedMode(double limit) {
            this.limit = limit;
        }

        /**
         * Gets the limit of this speed mode.
         * 
         * @param limit the current limit
         */
        public double getLimit() {
            return limit;
        }

        /**
         * Sets the limit of this speed mode.
         * 
         * @param limit the new limit
         */
        public void setLimit(double limit) {
            this.limit = limit;
        }
    }

    /** The speed that the robot is set to go at. */
    private SpeedMode speedMode = SpeedMode.ULTRA;

    /** Initializes the drivetrain. */
    public Drivetrain() {
        zeroGyro();
        poseEstimator = new SwerveDrivePoseEstimator(
                Constants.Geometries.Drivetrain.KINEMATICS,
                getGyroRotation2d(),
                getSwerveModulePositions(),
                new Pose2d(1.84, 5.06, new Rotation2d()));

        turnController.setTolerance(0.05);
        turnController.enableContinuousInput(0, 2 * Math.PI);

        LoggingManager.getInstance().addGroup("Drivetrain", new LogGroup(
                new DeviceLogger<Pigeon2>(pigeon, "Pigeon 2",
                        LogProfileBuilder.buildPigeon2LogItems(pigeon)),
                new DoubleLogItem("Slow Speed Limit", () -> SpeedMode.SLOW.getLimit(),
                        LogLevel.MAIN),
                new DoubleLogItem("Fast Speed Limit", () -> SpeedMode.FAST.getLimit(),
                        LogLevel.MAIN),
                new DoubleLogItem("Ultra Fast Speed Limit", () -> SpeedMode.ULTRA.getLimit(),
                        LogLevel.MAIN)));

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
     * Returns the current speed mode of the swerve (slow, fast, or ultra-fast).
     * 
     * @return the speed drive mode
     */
    public SpeedMode getSpeedMode() {
        return speedMode;
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
                        thetaSpeed, poseEstimator.getEstimatedPosition().getRotation());

                break;
        }
        SwerveModuleState[] states = Constants.Geometries.Drivetrain.KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states,
                Constants.Geometries.Drivetrain.MAX_PHYSICAL_VELOCITY_METERS_PER_SECOND);
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
     * Creates a command that zeros the gyro in whichever direction the bot is
     * pointing. Only use this if
     * the bot is straight, otherwise it will throw off field-oriented control.
     * 
     * @return the command
     */
    public CommandBase zeroGyroCommand() {
        return runOnce(() -> pigeon.setYaw(0));
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
        drawRobotOnField(AutoManager.getInstance().getField());
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
        return isTurningEnabled;
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
     * Createas a command that sets the current speed mode of the swerve (slow,
     * fast, or ultra-fast).
     * 
     * @param speedMode the speed mode to set
     */
    public CommandBase setSpeedModeCommand(SpeedMode speedMode) {
        return runOnce(() -> this.speedMode = speedMode);
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
            DoubleSupplier thetaSpeedSupplier, DoubleSupplier brakeSupplier, BooleanSupplier isInputCubed) {
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

            xSpeed *= speedMode.limit * brakeSupplier.getAsDouble();
            ySpeed *= speedMode.limit * brakeSupplier.getAsDouble();
            thetaSpeed *= speedMode.limit * brakeSupplier.getAsDouble();

            if (getTurnControllerEnabled()) {
                thetaSpeed = getTurnControllerOutput();
            }

            // the speeds are initially values from -1.0 to 1.0, so we multiply by the max
            // physical velocity to output in m/s.
            xSpeed *= Constants.Geometries.Drivetrain.MAX_PHYSICAL_VELOCITY_METERS_PER_SECOND;
            ySpeed *= Constants.Geometries.Drivetrain.MAX_PHYSICAL_VELOCITY_METERS_PER_SECOND;
            thetaSpeed *= Constants.Geometries.Drivetrain.MAX_PHYSICAL_VELOCITY_METERS_PER_SECOND;

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
    public CommandBase turnWhileMovingCommand(double angle) {
        return Commands.startEnd(() -> {
            if (!isTurningEnabled) {
                turnController.reset(getGyroAngleRad());
            }
            isTurningEnabled = true;
            turnController.setGoal(angle);
        }, () -> {
            isTurningEnabled = false;
        }).withName("Turn While Moving");
    }

    /**
     * Creates a command that sets the wheels to a "brake" configuration, where the
     * lateral side of each wheel is facing the center.
     * 
     * @return the command
     */
    public CommandBase brakeCommand() {
        return new InstantCommand(() -> {
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
        return new InstantCommand(() -> {
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
        return new InstantCommand(() -> {
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
    public CommandBase moveDeltaPathFollowingCommand(Transform2d delta, PathConstraints constraints) {
        return new ProxyCommand(
                () -> pathFollowingCommand(PathPlanner.generatePath(
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

    /**
     * Creates a command that supplies SwerveModuleStates to follow a
     * PathPlannerTrajectory, but the holonomic rotation is overridden to always be
     * π rad/s.
     * 
     * @return the command
     */
    public CommandBase chargeStationBalanceCommand() {
        PIDController controller = new PIDController(0.04, 0, 0.008);
        controller.setTolerance(1);
        return new PIDCommand(
                controller,
                () -> (-pigeon.getRoll()),
                0,
                (d) -> drive(
                        d > 0.84
                                ? (d > 0 ? 0.84 : -0.84)
                                : d,
                        0, 0, DriveMode.ROBOT_RELATIVE),
                this).finallyDo((d) -> this.stop());
    }

    // public CommandBase motorOverride(Motor) {
    // return new FunctionalCommand(
    // () -> {
    // driveMotor.setIdleMode(IdleMode.kCoast);
    // },
    // () -> {
    // driveMotor.stopMotor();
    // },
    // (d) -> {
    // driveMotor.setIdleMode(IdleMode.kBrake);
    // },
    // () -> false).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    // }

    public SwerveDrivePoseEstimator getPoseEstimator() {
        return this.poseEstimator;
    }

    public CommandBase setCoastCommand() {
        return Commands.runOnce(() -> {
            frontLeft.setIdleMode(IdleMode.kCoast);
            frontRight.setIdleMode(IdleMode.kCoast);
            backLeft.setIdleMode(IdleMode.kCoast);
            backRight.setIdleMode(IdleMode.kCoast);
        }).finallyDo(d -> {
            frontLeft.setIdleMode(IdleMode.kBrake);
            frontRight.setIdleMode(IdleMode.kBrake);
            backLeft.setIdleMode(IdleMode.kBrake);
            backRight.setIdleMode(IdleMode.kBrake);
        });
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
