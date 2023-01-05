package frc.robot.utils;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import com.techhounds.houndutil.houndlog.LogGroup;
import com.techhounds.houndutil.houndlog.LogProfileBuilder;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.enums.LogLevel;
import com.techhounds.houndutil.houndlog.loggers.Logger;
import com.techhounds.houndutil.houndlog.logitems.DoubleLogItem;
import com.techhounds.houndutil.houndlog.loggers.DeviceLogger;
import frc.robot.Constants;

public class SwerveModule {
    /** The motor used for driving. */
    private CANSparkMax driveMotor;
    /** The motor used for turning. */
    private CANSparkMax turnMotor;

    /** The encoder on the motor used for driving. */
    private RelativeEncoder driveEncoder;
    /** The CANCoder used to tell the angle of the wheel. */
    private CANCoder turnCanCoder;

    /** The PID controller that corrects the drive motor's velocity. */
    private PIDController drivePIDController = new PIDController(Constants.Drivetrain.PID.DriveMotors.kP,
            Constants.Drivetrain.PID.DriveMotors.kI, Constants.Drivetrain.PID.DriveMotors.kD);

    /** The PID controller that controls the turning motor's position. */
    private ProfiledPIDController turnPIDController = new ProfiledPIDController(
            Constants.Drivetrain.PID.TurnMotors.kP,
            Constants.Drivetrain.PID.TurnMotors.kI, Constants.Drivetrain.PID.TurnMotors.kD,
            new TrapezoidProfile.Constraints(
                    Constants.Drivetrain.Geometry.Turning.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                    Constants.Drivetrain.Geometry.Turning.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED));

    private PIDController turnPIDControllerSimple = new PIDController(Constants.Drivetrain.PID.TurnMotors.kP,
            Constants.Drivetrain.PID.TurnMotors.kI, Constants.Drivetrain.PID.TurnMotors.kD);

    /** The feedforward controller that controls the drive motor's velocity. */
    private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(
            Constants.Drivetrain.PID.DriveMotors.kS,
            Constants.Drivetrain.PID.DriveMotors.kV);

    private double drivePIDControllerOutput = 0.0;
    private double driveFFControllerOutput = 0.0;
    private double driveMotorVel = 0.0;
    private double driveMotorVelCF = 0.0;

    /**
     * The offset of the CANCoder from the zero point, in radians. This will be
     * *added* to any measurements obtained from the CANCoder.
     */
    private double turnCanCoderOffset;

    /**
     * Initalizes a SwerveModule.
     * 
     * @param name                 the name of the module (used for logging)
     * @param driveMotorChannel    the CAN ID of the drive motor
     * @param turnMotorChannel     the CAN ID of the turning motor
     * @param canCoderChannel      the CAN ID of the CANCoder
     * @param driveMotorInverted   if the drive motor is inverted
     * @param turnMotorInverted    if the turn motor is inverted
     * @param turnCanCoderInverted if the turn encoder is inverted
     * @param turnCanCoderOffset   the offset, in radians, to add to the CANCoder
     *                             value to make it zero when the module is straight
     */
    public SwerveModule(
            String name,
            int driveMotorChannel,
            int turnMotorChannel,
            int canCoderChannel,
            boolean driveMotorInverted,
            boolean turnMotorInverted,
            boolean turnCanCoderInverted,
            double turnCanCoderOffset) {

        driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
        driveMotor.restoreFactoryDefaults();
        driveMotor.setInverted(driveMotorInverted);
        driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        driveMotor.setSmartCurrentLimit(40);

        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(Constants.Drivetrain.Geometry.ENCODER_DISTANCE_TO_METERS);
        driveEncoder.setVelocityConversionFactor(Constants.Drivetrain.Geometry.ENCODER_DISTANCE_TO_METERS / 60.0);

        driveMotor.burnFlash();

        turnMotor = new CANSparkMax(turnMotorChannel, MotorType.kBrushless);
        turnMotor.restoreFactoryDefaults();
        turnMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        turnMotor.setInverted(turnMotorInverted);
        turnMotor.setSmartCurrentLimit(15);
        turnMotor.burnFlash();

        turnCanCoder = new CANCoder(canCoderChannel);

        // There is an issue with absolute position vs position in CANCoders, namely
        // that the abs pos is sent a lot less frequently than the normal pos (every
        // 100ms vs every 10ms). According to this post:
        // https://www.chiefdelphi.com/t/official-sds-mk3-mk4-code/397109/99, setting
        // the CANCoder to "Boot to Absolute" will fix this.
        turnCanCoder.setPositionToAbsolute();
        turnCanCoder.configSensorDirection(turnCanCoderInverted);
        turnCanCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        turnCanCoder.configFeedbackCoefficient(2 * Math.PI / 4096.0, "rad", SensorTimeBase.PerSecond); // radians/sec

        turnPIDController.enableContinuousInput(0, 2 * Math.PI);
        turnPIDControllerSimple.enableContinuousInput(0, 2 * Math.PI);

        this.turnCanCoderOffset = turnCanCoderOffset;

        LoggingManager.getInstance().addGroup(name, new LogGroup(
                new Logger[] {
                        new DeviceLogger<CANSparkMax>(driveMotor, "Drive Motor",
                                LogProfileBuilder.buildCANSparkMaxLogItems(driveMotor)),
                        new DeviceLogger<CANSparkMax>(turnMotor, "Turning Motor",
                                LogProfileBuilder.buildCANSparkMaxLogItems(turnMotor)),
                        new DeviceLogger<CANCoder>(turnCanCoder, "CANCoder",
                                LogProfileBuilder.buildCANCoderLogItems(turnCanCoder)),
                        new DoubleLogItem("Wheel Angle", this::getAngle, LogLevel.MAIN),
                        new DoubleLogItem("CANCoder Position", turnCanCoder::getPosition, LogLevel.MAIN),
                        new DoubleLogItem("Wheel Angle", this::getAngle, LogLevel.MAIN),
                        new DoubleLogItem("CANCoder Position", turnCanCoder::getPosition, LogLevel.MAIN),
                        new DoubleLogItem("Drive PID", () -> this.drivePIDControllerOutput, LogLevel.MAIN),
                        new DoubleLogItem("Drive FF", () -> this.driveFFControllerOutput, LogLevel.MAIN),
                        new DoubleLogItem("Drive Motor Vel", () -> this.driveMotorVel, LogLevel.MAIN),
                        new DoubleLogItem("Drive Motor Vel CF", () -> this.driveMotorVelCF, LogLevel.MAIN)

                }));
    }

    /**
     * Gets the state of the swerve module.
     * 
     * @return the state of the swerve module
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), new Rotation2d(getWheelAngle()));
    }

    /**
     * Gets the state of the swerve module.
     * 
     * @return the state of the swerve module
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(getWheelAngle()));
    }

    /**
     * Sets the PID controller setpoints to the desired state.
     * 
     * @param state    the desired state of the swerve module
     * @param openLoop whether to run open loop controls
     * @param optimize whether to optimize the state of the modules
     */
    public void setState(SwerveModuleState state, boolean openLoop, boolean optimize) {
        if (state.speedMetersPerSecond < 0.01) {
            stop();
            return;
        }

        if (optimize)
            state = SwerveModuleState.optimize(state, new Rotation2d(getWheelAngle()));

        if (openLoop) {
            driveMotor.setVoltage(state.speedMetersPerSecond
                    / Constants.Drivetrain.Geometry.MAX_PHYSICAL_VELOCITY_METERS_PER_SECOND * 12.0);

            double v = turnPIDControllerSimple.calculate(getWheelAngle(),
                    state.angle.getRadians()) * 12.0;
            if (v > 8)
                v = 8.0;

            turnMotor.setVoltage(v);
        } else {
            this.drivePIDControllerOutput = drivePIDController.calculate(driveEncoder.getVelocity(),
                    state.speedMetersPerSecond);
            this.driveFFControllerOutput = driveFeedforward.calculate(state.speedMetersPerSecond);
            this.driveMotorVel = driveEncoder.getVelocity();
            this.driveMotorVelCF = driveEncoder.getVelocityConversionFactor();

            double turnOutput = turnPIDController.calculate(getWheelAngle(),
                    state.angle.getRadians());

            driveMotor.setVoltage(drivePIDControllerOutput + driveFFControllerOutput);
            turnMotor.set(turnOutput);
        }
    }

    /**
     * Sets the PID controller setpoints to the desired state.
     * Defaults to open loop controls and optimized state.
     * 
     * @param state the desired state of the swerve module
     */
    public void setState(SwerveModuleState state) {
        setState(state, true, true);
    }

    /**
     * Sets the rotation of the module. X and Y velocities will be ignored.
     * 
     * @param angle the angle of the wheel in radians
     */
    public void setRotation(double angle) {
        turnMotor.set(turnPIDControllerSimple.calculate(getWheelAngle(),
                angle));
    }

    /**
     * Gets the CANCoder position, adjusted with the offset.
     * 
     * @return the position of the CANCoder
     */
    public double getWheelAngle() {
        return turnCanCoder.getPosition() + turnCanCoderOffset;
    }

    /**
     * Gets the position of the drive encoder.
     * 
     * @return the position of the drive encoder.
     */
    public double getDriveEncoderPosition() {
        return driveEncoder.getPosition();
    }

    /**
     * Reset the position of the encoder on the drive motor.
     */
    public void resetDriveEncoder() {
        driveEncoder.setPosition(0);
    }

    /**
     * Gets the current angle of the wheel.
     * 
     * @return the current angle of the wheel, in degrees, [0, 360) CCW.
     */
    public double getAngle() {
        return Math.toDegrees(turnCanCoder.getPosition());
    }

    /**
     * Stops the swerve module.
     */
    public void stop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }
}