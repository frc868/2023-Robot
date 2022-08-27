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
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.houndutil.houndlog.LogGroup;
import frc.houndutil.houndlog.LogProfileBuilder;
import frc.houndutil.houndlog.LoggingManager;
import frc.houndutil.houndlog.enums.LogLevel;
import frc.houndutil.houndlog.enums.LogType;
import frc.houndutil.houndlog.loggers.Logger;
import frc.houndutil.houndlog.loggers.SingleItemLogger;
import frc.houndutil.houndlog.loggers.DeviceLogger;
import frc.robot.Constants;

public class SwerveModule {
    /** The motor used for driving. */
    private CANSparkMax driveMotor;
    /** The motor used for turning. */
    private CANSparkMax turnMotor;

    /** The encoder on the motor used for driving. */
    private RelativeEncoder driveEncoder;
    /** The CANCoder used to tell the angle of the wheel. */
    private CANCoder turnEncoder;

    /** The PID controller that corrects the drive motor's velocity. */
    private PIDController drivePIDController = new PIDController(Constants.Drivetrain.PID.Drive.kP,
            Constants.Drivetrain.PID.Drive.kI, Constants.Drivetrain.PID.Drive.kD);

    /** The PID controller that controls the turning motor's position. */
    private ProfiledPIDController turnPIDController = new ProfiledPIDController(
            Constants.Drivetrain.PID.Turn.kP,
            Constants.Drivetrain.PID.Turn.kI, Constants.Drivetrain.PID.Turn.kD,
            new TrapezoidProfile.Constraints(Constants.Drivetrain.Geometry.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                    Constants.Drivetrain.Geometry.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED));

    /** The feedforward controller that controls the drive motor's velocity. */
    private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(Constants.Drivetrain.PID.Drive.kS,
            Constants.Drivetrain.PID.Drive.kV);

    /**
     * Initalizes a SwerveModule.
     * 
     * @param name                the name of the module (used for logging)
     * @param driveMotorChannel   the CAN ID of the drive motor
     * @param turnMotorChannel    the CAN ID of the turning motor
     * @param canCoderChannel     the CAN ID of the CANCoder
     * @param driveMotorInverted  if the drive motor is inverted
     * @param turnEncoderInverted if the turn encoder is inverted
     */
    public SwerveModule(
            String name,
            int driveMotorChannel,
            int turnMotorChannel,
            int canCoderChannel,
            boolean driveMotorInverted,
            boolean turnEncoderInverted) {

        driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
        driveMotor.setInverted(driveMotorInverted);
        driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        turnMotor = new CANSparkMax(turnMotorChannel, MotorType.kBrushless);
        turnMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(2 * Math.PI * Constants.Drivetrain.Geometry.WHEEL_RADIUS_METERS); // in
                                                                                                                   // meters
        driveEncoder
                .setVelocityConversionFactor((2 * Math.PI * Constants.Drivetrain.Geometry.WHEEL_RADIUS_METERS) / 60.0);

        turnEncoder = new CANCoder(canCoderChannel);

        // There is an issue with absolute position vs position in CANCoders, namely
        // that the abs pos is sent a lot less frequently than the normal pos (every
        // 100ms vs every 10ms). According to this post:
        // https://www.chiefdelphi.com/t/official-sds-mk3-mk4-code/397109/99, setting
        // the CANCoder to "Boot to Absolute" will fix this.
        turnEncoder.setPositionToAbsolute();
        turnEncoder.configSensorDirection(turnEncoderInverted);
        turnEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        turnEncoder.configFeedbackCoefficient(2 * Math.PI / 4096.0, "rad", SensorTimeBase.PerSecond); // radians/sec

        turnPIDController.enableContinuousInput(-Math.PI, Math.PI);

        LoggingManager.getInstance().addGroup(name, new LogGroup(
                new Logger[] {
                        new DeviceLogger<CANSparkMax>(driveMotor, "Drive Motor",
                                LogProfileBuilder.buildCANSparkMaxLogItems(driveMotor)),
                        new DeviceLogger<CANSparkMax>(turnMotor, "Turning Motor",
                                LogProfileBuilder.buildCANSparkMaxLogItems(turnMotor)),
                        new SingleItemLogger<Double>(LogType.NUMBER, "Wheel Angle", this::getAngle, LogLevel.MAIN)
                }));
    }

    /**
     * Gets the state of the swerve module.
     * 
     * @return the state of the swerve module
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(turnEncoder.getPosition()));
    }

    /**
     * Sets the PID controller setpoints to the desired state.
     * 
     * @param state the desired state of the swerve module
     */
    public void setState(SwerveModuleState state) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, new Rotation2d(turnEncoder.getPosition()));
        double driveOutput = drivePIDController.calculate(driveEncoder.getVelocity(),
                optimizedState.speedMetersPerSecond)
                + driveFeedforward.calculate(optimizedState.speedMetersPerSecond);
        double turnOutput = turnPIDController.calculate(turnEncoder.getPosition(), optimizedState.angle.getRadians());

        driveMotor.set(driveOutput);
        turnMotor.set(turnOutput);
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
        return Math.toDegrees(turnEncoder.getPosition());
    }
}