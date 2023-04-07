package frc.robot.utils;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.techhounds.houndutil.houndlib.SparkMaxConfigurator;
import com.techhounds.houndutil.houndlog.LogGroup;
import com.techhounds.houndutil.houndlog.LogProfileBuilder;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.enums.LogLevel;
import com.techhounds.houndutil.houndlog.loggers.DeviceLogger;
import com.techhounds.houndutil.houndlog.loggers.Logger;
import com.techhounds.houndutil.houndlog.logitems.DoubleLogItem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;
import frc.robot.Overrides;

public class SwerveModule {
    /** The motor used for driving. */
    private CANSparkMax driveMotor;
    /** The motor used for turning. */
    private CANSparkMax turnMotor;

    /** The encoder on the motor used for driving. */
    private RelativeEncoder driveEncoder;
    /** The encoder on the motor used for turning. */
    private RelativeEncoder turnEncoder;
    /** The CANCoder used to tell the angle of the wheel. */
    private CANCoder turnCanCoder;

    /** The PID controller that corrects the drive motor's velocity. */
    private PIDController drivePIDController = new PIDController(Constants.Gains.DriveMotors.kP.get(),
            Constants.Gains.DriveMotors.kI.get(), Constants.Gains.DriveMotors.kD.get());

    /** The PID controller that controls the turning motor's position. */
    private ProfiledPIDController turnPIDController = new ProfiledPIDController(
            Constants.Gains.TurnMotors.kP.get(),
            Constants.Gains.TurnMotors.kI.get(), Constants.Gains.TurnMotors.kD.get(),
            new TrapezoidProfile.Constraints(
                    Constants.Geometries.Drivetrain.Turning.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                    Constants.Geometries.Drivetrain.Turning.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED));

    private PIDController turnPIDControllerSimple = new PIDController(Constants.Gains.TurnMotors.kP.get(),
            Constants.Gains.TurnMotors.kI.get(), Constants.Gains.TurnMotors.kD.get());

    /** The feedforward controller that controls the drive motor's velocity. */
    private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(
            Constants.Gains.DriveMotors.kS,
            Constants.Gains.DriveMotors.kV,
            Constants.Gains.DriveMotors.kA);

    private double setpointVelocity = 0.0;
    private double setpointAngle = 0.0;
    private double drivePidOutput = 0.0;
    private double driveFeedforwardOutput = 0.0;
    private double turnPidOutput = 0.0;

    private double simDriveEncoderPosition;
    private double simDriveEncoderVelocity;
    private double simCurrentAngle;

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
        SparkMaxConfigurator.configure(driveMotor)
                .withIdleMode(IdleMode.kBrake)
                .withInverted(driveMotorInverted)
                .withCurrentLimit(40)
                .withPositionConversionFactor(Constants.Geometries.Drivetrain.ENCODER_DISTANCE_TO_METERS,
                        true)
                .burnFlash();

        driveEncoder = driveMotor.getEncoder();

        turnMotor = new CANSparkMax(turnMotorChannel, MotorType.kBrushless);
        SparkMaxConfigurator.configure(turnMotor)
                .withIdleMode(IdleMode.kBrake)
                .withInverted(turnMotorInverted)
                .withPositionConversionFactor(Constants.Geometries.Drivetrain.TURN_ENCODER_DISTANCE_TO_METERS, true)
                .withCurrentLimit(15).burnFlash();

        turnEncoder = turnMotor.getEncoder();

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

        new Thread(() -> {
            try {
                Thread.sleep(3000);
                turnEncoder.setPosition(turnCanCoder.getPosition() + turnCanCoderOffset);
            } catch (Exception e) {
            }
        }).start();

        turnPIDController.enableContinuousInput(0, 2 * Math.PI);
        turnPIDControllerSimple.enableContinuousInput(0, 2 * Math.PI);

        this.turnCanCoderOffset = turnCanCoderOffset;

        Constants.Gains.DriveMotors.kP.setConsumer((d) -> drivePIDController.setP(d));
        Constants.Gains.DriveMotors.kI.setConsumer((d) -> drivePIDController.setI(d));
        Constants.Gains.DriveMotors.kD.setConsumer((d) -> drivePIDController.setD(d));
        Constants.Gains.TurnMotors.kP.setConsumer((d) -> turnPIDController.setP(d));
        Constants.Gains.TurnMotors.kI.setConsumer((d) -> turnPIDController.setI(d));
        Constants.Gains.TurnMotors.kD.setConsumer((d) -> turnPIDController.setD(d));

        LoggingManager.getInstance().addGroup(name, new LogGroup(
                new Logger[] {
                        new DeviceLogger<CANSparkMax>(driveMotor, "Drive Motor",
                                LogProfileBuilder.buildCANSparkMaxLogItems(driveMotor)),
                        new DeviceLogger<CANSparkMax>(turnMotor, "Turning Motor",
                                LogProfileBuilder.buildCANSparkMaxLogItems(turnMotor)),
                        new DeviceLogger<CANCoder>(turnCanCoder, "CANCoder",
                                LogProfileBuilder.buildCANCoderLogItems(turnCanCoder)),
                        new DoubleLogItem("Wheel Angle Degrees", () -> Units.radiansToDegrees(getWheelAngle()),
                                LogLevel.MAIN),
                        new DoubleLogItem("Control/Setpoint Velocity", () -> this.setpointVelocity, LogLevel.MAIN),
                        new DoubleLogItem("Control/Setpoint Angle", () -> this.setpointAngle, LogLevel.MAIN),
                        new DoubleLogItem("Control/Drive PID Output", () -> this.drivePidOutput, LogLevel.MAIN),
                        new DoubleLogItem("Control/Drive Feedforward Output", () -> this.driveFeedforwardOutput,
                                LogLevel.MAIN),
                        new DoubleLogItem("Control/Total Drive Output",
                                () -> this.drivePidOutput + this.driveFeedforwardOutput, LogLevel.MAIN),
                        new DoubleLogItem("Control/Turn PID Output", () -> this.turnPidOutput, LogLevel.MAIN),
                        new DoubleLogItem("Control/Turn PID Output", () -> this.turnPidOutput, LogLevel.MAIN),

                }));

        if (RobotBase.isSimulation()) {
            this.simDriveEncoderPosition = 0.0;
            this.simDriveEncoderVelocity = 0.0;
            this.simCurrentAngle = 0.0;
        }
    }

    /**
     * Gets the CANCoder position, adjusted with the offset.
     * 
     * @return the position of the CANCoder
     */
    public double getWheelAngle() {
        if (RobotBase.isReal())
            return Overrides.ABSOLUTE_ENCODERS.getStatus() ? turnCanCoder.getPosition() + turnCanCoderOffset
                    : turnEncoder.getPosition();
        else
            return simCurrentAngle;
    }

    /**
     * Gets the position of the drive encoder.
     * 
     * @return the position of the drive encoder.
     */
    public double getDriveEncoderPosition() {
        if (RobotBase.isReal())
            return driveEncoder.getPosition();
        else
            return simDriveEncoderPosition;
    }

    /**
     * Gets the velocity of the drive encoder.
     * 
     * @return the velocity of the drive encoder.
     */
    public double getDriveEncoderVelocity() {
        if (RobotBase.isReal())
            return driveEncoder.getVelocity();
        else
            return simDriveEncoderVelocity;
    }

    /**
     * Gets the state of the swerve module.
     * 
     * @return the state of the swerve module
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDriveEncoderPosition(), new Rotation2d(getWheelAngle()));
    }

    /**
     * Gets the state of the swerve module.
     * 
     * @return the state of the swerve module
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveEncoderVelocity(), new Rotation2d(getWheelAngle()));
    }

    /**
     * Sets the PID controller setpoints to the desired state.
     * 
     * @param state    the desired state of the swerve module
     * @param openLoop whether to run open loop controls
     * @param optimize whether to optimize the state of the modules
     */
    public void setState(SwerveModuleState state, boolean openLoop, boolean optimize) {
        if (optimize)
            state = SwerveModuleState.optimize(state, new Rotation2d(getWheelAngle()));

        if (openLoop) {
            driveMotor.setVoltage(state.speedMetersPerSecond
                    / Constants.Geometries.Drivetrain.MAX_PHYSICAL_VELOCITY_METERS_PER_SECOND * 12.0);

            double v = turnPIDControllerSimple.calculate(getWheelAngle(),
                    state.angle.getRadians()) * 12.0;
            if (v > 8)
                v = 8.0; // drops voltage down to 8

            if (RobotBase.isReal()) {
                turnMotor.setVoltage(v);
            } else {
                simDriveEncoderVelocity = state.speedMetersPerSecond;
                double distancePer20Ms = state.speedMetersPerSecond / 50.0;
                simDriveEncoderPosition += distancePer20Ms;

                simCurrentAngle = state.angle.getRadians();
                turnCanCoder.setPosition(simCurrentAngle);
            }

        } else {
            this.drivePidOutput = drivePIDController.calculate(driveEncoder.getVelocity(),
                    state.speedMetersPerSecond);
            this.driveFeedforwardOutput = driveFeedforward.calculate(state.speedMetersPerSecond);
            this.turnPidOutput = turnPIDControllerSimple.calculate(getWheelAngle(),
                    state.angle.getRadians());

            driveMotor.setVoltage(drivePidOutput + driveFeedforwardOutput);
            turnMotor.set(turnPidOutput);
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
     * Reset the position of the encoder on the drive motor.
     */
    public void resetDriveEncoder() {
        driveEncoder.setPosition(0);
    }

    /**
     * Stops the swerve module.
     */
    public void stop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }

    public void setIdleMode(IdleMode idleMode) {
        driveMotor.setIdleMode(idleMode);
        turnMotor.setIdleMode(idleMode);
    }

    public void setDriveCurrentLimit(int currentLimit) {
        driveMotor.setSmartCurrentLimit(currentLimit);
    }
}