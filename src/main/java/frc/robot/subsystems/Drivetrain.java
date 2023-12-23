package frc.robot.subsystems;

import java.util.Set;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.techhounds.houndutil.houndauto.AutoManager;
import com.techhounds.houndutil.houndlib.MotorHoldMode;
import com.techhounds.houndutil.houndlib.subsystems.BaseSwerveDrive;
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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

import static frc.robot.Constants.Drivetrain.*;
import static frc.robot.Constants.Teleop.*;

/**
 * The drivetrain subsystem, containing four swerve modules and odometry-related
 * information.
 * 
 * @author dr
 */
@LoggedObject
public class Drivetrain extends SubsystemBase implements BaseSwerveDrive {
    /** The front left swerve module when looking at the bot from behind. */
    @Log(groups = "modules")
    private NEOCoaxialSwerveModule frontLeft = new NEOCoaxialSwerveModule(
            FRONT_LEFT_DRIVE_MOTOR_ID,
            FRONT_LEFT_STEER_MOTOR_ID,
            FRONT_LEFT_STEER_ENCODER_ID,
            false,
            true,
            false,
            FRONT_LEFT_OFFSET,
            SWERVE_CONSTANTS);

    /** The front right swerve module when looking at the bot from behind. */
    @Log(groups = "modules")
    private NEOCoaxialSwerveModule frontRight = new NEOCoaxialSwerveModule(
            FRONT_RIGHT_DRIVE_MOTOR_ID,
            FRONT_RIGHT_STEER_MOTOR_ID,
            FRONT_RIGHT_STEER_ENCODER_ID,
            false,
            true,
            false,
            FRONT_RIGHT_OFFSET,
            SWERVE_CONSTANTS);

    /** The back left swerve module when looking at the bot from behind. */
    @Log(groups = "modules")
    private NEOCoaxialSwerveModule backLeft = new NEOCoaxialSwerveModule(
            BACK_LEFT_DRIVE_MOTOR_ID,
            BACK_LEFT_STEER_MOTOR_ID,
            BACK_LEFT_STEER_ENCODER_ID,
            false,
            true,
            false,
            BACK_LEFT_OFFSET,
            SWERVE_CONSTANTS);

    /** The back right swerve module when looking at the bot from behind. */
    @Log(groups = "modules")
    private NEOCoaxialSwerveModule backRight = new NEOCoaxialSwerveModule(
            BACK_RIGHT_DRIVE_MOTOR_ID,
            BACK_RIGHT_STEER_MOTOR_ID,
            BACK_RIGHT_STEER_ENCODER_ID,
            false,
            true,
            false,
            BACK_RIGHT_OFFSET,
            SWERVE_CONSTANTS);

    /** The gyroscope on the robot. */
    @Log
    private Pigeon2 pigeon = new Pigeon2(0);

    /**
     * The pose estimator, which takes in wheel odometry and latency-compensated
     * AprilTag measurements to provide an absolute position on the field.
     */
    @Log
    private SwerveDrivePoseEstimator poseEstimator;

    @Log
    private ProfiledPIDController xPositionController = new ProfiledPIDController(
            XY_kP, XY_kI, XY_kD, XY_CONSTRAINTS);

    @Log
    private ProfiledPIDController yPositionController = new ProfiledPIDController(
            XY_kP, XY_kI, XY_kD, XY_CONSTRAINTS);

    @Log
    private ProfiledPIDController thetaPositionController = new ProfiledPIDController(
            THETA_kP, THETA_kI, THETA_kD, THETA_CONSTRAINTS);

    @Log(groups = "control")
    private SwerveModuleState[] commandedModuleStates = new SwerveModuleState[] { new SwerveModuleState(),
            new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState() };

    @Log(groups = "control")
    private ChassisSpeeds commandedChassisSpeeds = new ChassisSpeeds();

    /** The mode of driving, either robot relative or field relative. */
    @Log
    private DriveMode driveMode = DriveMode.FIELD_ORIENTED;

    /**
     * The yaw of the gyro in simulations. This value is not used when the robot is
     * running IRL.
     */
    private double simYaw;

    /**
     * Whether to override the inputs of the driver for maintaining or turning to a
     * specific angle.
     */
    @Log
    private boolean isControlledRotationEnabled = false;

    private SwerveDriveOdometry simOdometry;

    /** Initializes the drivetrain. */
    public Drivetrain() {
        resetGyro();
        poseEstimator = new SwerveDrivePoseEstimator(
                KINEMATICS,
                getRotation(),
                getModulePositions(),
                new Pose2d(0, 0, new Rotation2d()));

        thetaPositionController.setTolerance(0.05);
        thetaPositionController.enableContinuousInput(0, 2 * Math.PI);

        AutoManager.getInstance().setResetOdometryConsumer(this::resetPoseEstimator);

        if (RobotBase.isSimulation()) {
            simYaw = 0;
            simOdometry = new SwerveDriveOdometry(KINEMATICS, getRotation(), getModulePositions(), new Pose2d());
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
        simYaw += getChassisSpeeds().omegaRadiansPerSecond * 0.020;

        pigeon.getSimState().setRawYaw(Units.radiansToDegrees(simYaw));

        simOdometry.update(getRotation(), getModulePositions());
    }

    @Override
    public DriveMode getDriveMode() {
        return driveMode;
    }

    @Override
    @Log
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    @Log
    public Pose2d getSimPose() {
        return simOdometry.getPoseMeters();
    }

    @Override
    public Rotation2d getRotation() {
        return Rotation2d.fromDegrees(pigeon.getYaw().getValue());
    }

    @Override
    @Log(groups = "control")
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
        };
    }

    @Override
    @Log(groups = "control")
    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState()
        };
    }

    @Override
    @Log(groups = "control")
    public ChassisSpeeds getChassisSpeeds() {
        return KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    @Override
    public SwerveDrivePoseEstimator getPoseEstimator() {
        return poseEstimator;
    }

    @Override
    public void updatePoseEstimator() {
        poseEstimator.update(getRotation(), getModulePositions());
        drawRobotOnField(AutoManager.getInstance().getField());
    }

    @Override
    public void resetPoseEstimator(Pose2d pose) {
        poseEstimator.resetPosition(getRotation(), getModulePositions(), pose);
        pigeon.setYaw(pose.getRotation().getDegrees());
    }

    @Override
    public void resetGyro() {
        pigeon.setYaw(0);
    }

    @Override
    public void setMotorHoldModes(MotorHoldMode motorHoldMode) {
        frontLeft.setMotorHoldMode(motorHoldMode);
        frontRight.setMotorHoldMode(motorHoldMode);
        backLeft.setMotorHoldMode(motorHoldMode);
        backRight.setMotorHoldMode(motorHoldMode);
    }

    @Override
    public void setDriveCurrentLimit(int currentLimit) {
        frontLeft.setDriveCurrentLimit(currentLimit);
        frontRight.setDriveCurrentLimit(currentLimit);
        backLeft.setDriveCurrentLimit(currentLimit);
        backRight.setDriveCurrentLimit(currentLimit);
    }

    @Override
    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    @Override
    public void setStates(SwerveModuleState[] states) {
        frontLeft.setState(states[0]);
        frontRight.setState(states[1]);
        backLeft.setState(states[2]);
        backRight.setState(states[3]);
    }

    @Override
    public void setStatesClosedLoop(SwerveModuleState[] states) {
        frontLeft.setStateClosedLoop(states[0]);
        frontRight.setStateClosedLoop(states[1]);
        backLeft.setStateClosedLoop(states[2]);
        backRight.setStateClosedLoop(states[3]);
    }

    public void drive(ChassisSpeeds speeds) {
        drive(speeds, this.driveMode);
    }

    @Override
    public void drive(ChassisSpeeds speeds, DriveMode driveMode) {
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red
                && driveMode == DriveMode.FIELD_ORIENTED) {
            speeds.vxMetersPerSecond *= -1;
            speeds.vyMetersPerSecond *= -1;
        }

        commandedChassisSpeeds = speeds;
        ChassisSpeeds adjustedChassisSpeeds = null;
        switch (driveMode) {
            case ROBOT_RELATIVE:
                adjustedChassisSpeeds = speeds;
                break;
            case FIELD_ORIENTED:
                adjustedChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds.vxMetersPerSecond,
                        speeds.vyMetersPerSecond,
                        speeds.omegaRadiansPerSecond, getRotation());

                break;
        }

        SwerveModuleState[] states = KINEMATICS.toSwerveModuleStates(adjustedChassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states,
                SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND);
        commandedModuleStates = states;
        setStates(states);
    }

    @Override
    public void driveClosedLoop(ChassisSpeeds speeds, DriveMode driveMode) {
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red
                && driveMode == DriveMode.FIELD_ORIENTED) {
            speeds.vxMetersPerSecond *= -1;
            speeds.vyMetersPerSecond *= -1;
        }

        commandedChassisSpeeds = new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond,
                speeds.omegaRadiansPerSecond);

        ChassisSpeeds chassisSpeeds = null;
        switch (driveMode) {
            case ROBOT_RELATIVE:
                chassisSpeeds = commandedChassisSpeeds;
                break;
            case FIELD_ORIENTED:
                chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds.vxMetersPerSecond,
                        speeds.vyMetersPerSecond,
                        speeds.omegaRadiansPerSecond, getRotation());

                break;
        }

        SwerveModuleState[] states = KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states,
                SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND);
        commandedModuleStates = states;
        setStatesClosedLoop(states);
    }

    @Override
    public Command teleopDriveCommand(DoubleSupplier xSpeedSupplier, DoubleSupplier ySpeedSupplier,
            DoubleSupplier thetaSpeedSupplier) {
        SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(JOYSTICK_INPUT_RATE_LIMIT);
        SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(JOYSTICK_INPUT_RATE_LIMIT);
        SlewRateLimiter thetaSpeedLimiter = new SlewRateLimiter(JOYSTICK_INPUT_RATE_LIMIT);

        return run(() -> {
            double xSpeed = xSpeedSupplier.getAsDouble();
            double ySpeed = ySpeedSupplier.getAsDouble();
            double thetaSpeed = thetaSpeedSupplier.getAsDouble();

            xSpeed = MathUtil.applyDeadband(xSpeed, 0.05);
            ySpeed = MathUtil.applyDeadband(ySpeed, 0.05);
            thetaSpeed = MathUtil.applyDeadband(thetaSpeed, 0.05);

            xSpeed = Math.pow(xSpeed, 3);
            ySpeed = Math.pow(ySpeed, 3);
            thetaSpeed = Math.pow(thetaSpeed, 3);

            xSpeed = xSpeedLimiter.calculate(xSpeed);
            ySpeed = ySpeedLimiter.calculate(ySpeed);
            thetaSpeed = thetaSpeedLimiter.calculate(thetaSpeed);

            if (isControlledRotationEnabled) {
                thetaSpeed = thetaPositionController.calculate(getRotation().getRadians());
            }

            // the speeds are initially values from -1.0 to 1.0, so we multiply by the max
            // physical velocity to output in m/s.
            xSpeed *= SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND;
            ySpeed *= SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND;
            thetaSpeed *= SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND;

            drive(new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed));
        }).withName("Teleop Drive Command");
    }

    @Override
    public Command controlledRotateCommand(double angle, DriveMode driveMode) {
        return Commands.startEnd(() -> {
            if (!isControlledRotationEnabled) {
                thetaPositionController.reset(getRotation().getRadians());
            }
            isControlledRotationEnabled = true;
            if (driveMode == DriveMode.FIELD_ORIENTED && DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == Alliance.Red)
                thetaPositionController.setGoal(angle + Math.PI);
            else
                thetaPositionController.setGoal(angle);
        }, () -> {
            isControlledRotationEnabled = false;
        }).withName("Turn While Moving");
    }

    @Override
    public Command wheelLockCommand() {
        return Commands.runOnce(() -> {
            setStates(new SwerveModuleState[] {
                    new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(45))
            });
        }).withName("Wheel Lock");
    }

    @Override
    public Command turnWheelsToAngleCommand(double angle) {
        return Commands.runOnce(() -> {
            setStates(new SwerveModuleState[] {
                    new SwerveModuleState(0, new Rotation2d(angle)),
                    new SwerveModuleState(0, new Rotation2d(angle)),
                    new SwerveModuleState(0, new Rotation2d(angle)),
                    new SwerveModuleState(0, new Rotation2d(angle))

            });
        }).withName("Turn Wheels To Angle");
    }

    @Override
    public Command driveToPoseCommand(Pose2d pose) {
        return runOnce(() -> {
            xPositionController.reset(getPose().getX());
            yPositionController.reset(getPose().getY());
            thetaPositionController.reset(getPose().getRotation().getRadians());
        }).andThen(run(() -> {
            driveClosedLoop(
                    new ChassisSpeeds(
                            xPositionController.calculate(getPose().getX(), pose.getX()),
                            yPositionController.calculate(getPose().getX(), pose.getY()),
                            thetaPositionController.calculate(getPose().getX(), pose.getRotation().getRadians())),
                    DriveMode.FIELD_ORIENTED);
        }));
    }

    @Override
    public Command pathFollowingCommand(PathPlannerPath path) {
        return new FollowPathHolonomic(
                path,
                this::getPose,
                this::getChassisSpeeds,
                (speeds) -> driveClosedLoop(speeds, DriveMode.ROBOT_RELATIVE),
                new HolonomicPathFollowerConfig(
                        new PIDConstants(5.0, 0.0, 0.0),
                        new PIDConstants(5.0, 0.0, 0.0),
                        4.5,
                        0.4,
                        new ReplanningConfig()),
                this);
    }

    @Override
    public Command driveDeltaPathFollowingCommand(Transform2d delta,
            PathConstraints constraints) {

        return new DeferredCommand(() -> pathFollowingCommand(
                new PathPlannerPath(
                        PathPlannerPath.bezierFromPoses(
                                getPose(), getPose().plus(delta)),
                        constraints,
                        new GoalEndState(0, delta.getRotation().plus(getRotation())))),
                Set.of());
    }

    @Override
    public Command setDriveModeCommand(DriveMode driveMode) {
        return runOnce(() -> this.driveMode = driveMode);
    }

    @Override
    public Command resetGyroCommand() {
        return runOnce(() -> resetGyro());
    }

    @Override
    public Command setDriveCurrentLimitCommand(int currentLimit) {
        return Commands.runOnce(() -> {
            frontLeft.setDriveCurrentLimit(currentLimit);
            frontRight.setDriveCurrentLimit(currentLimit);
            backLeft.setDriveCurrentLimit(currentLimit);
            backRight.setDriveCurrentLimit(currentLimit);
        });
    }

    @Override
    public Command coastMotorsCommand() {
        return runOnce(this::stop)
                .andThen(() -> setMotorHoldModes(MotorHoldMode.COAST))
                .finallyDo((d) -> setMotorHoldModes(MotorHoldMode.BRAKE))
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
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
                        new Transform2d(SWERVE_MODULE_LOCATIONS[0],
                                getModuleStates()[0].angle)));
        field.getObject("frontRight").setPose(
                getPose().transformBy(
                        new Transform2d(SWERVE_MODULE_LOCATIONS[1],
                                getModuleStates()[1].angle)));
        field.getObject("backLeft").setPose(
                getPose().transformBy(
                        new Transform2d(SWERVE_MODULE_LOCATIONS[2],
                                getModuleStates()[2].angle)));
        field.getObject("backRight").setPose(
                getPose().transformBy(
                        new Transform2d(SWERVE_MODULE_LOCATIONS[3],
                                getModuleStates()[3].angle)));
    }

    /**
     * Creates a command that supplies SwerveModuleStates to follow a
     * PathPlannerTrajectory, but the holonomic rotation is overridden to always be
     * π rad/s.
     * 
     * @return the command
     */
    public Command chargeStationBalanceCommand() {
        PIDController controller = new PIDController(BALANCE_kP, BALANCE_kI, BALANCE_kD);

        controller.setTolerance(BALANCE_TOLERANCE);
        return new PIDCommand(
                controller,
                () -> -pigeon.getRoll().getValue(),
                0,
                (d) -> drive(
                        new ChassisSpeeds(MathUtil.clamp(d, -BALANCE_MAX_VALUE, BALANCE_MAX_VALUE), 0, 0),
                        DriveMode.ROBOT_RELATIVE),
                this)
                .finallyDo((d) -> this.stop());
    }

}
