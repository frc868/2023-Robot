package frc.robot.utils;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
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
    private CANSparkMax driveMotor;
    private CANSparkMax turnMotor;

    private RelativeEncoder driveEncoder;
    private CANCoder turnEncoder;

    private PIDController drivePIDController = new PIDController(Constants.Drivetrain.PIDConstants.Drive.kP,
            Constants.Drivetrain.PIDConstants.Drive.kI, Constants.Drivetrain.PIDConstants.Drive.kD);

    private ProfiledPIDController turnPIDController = new ProfiledPIDController(
            Constants.Drivetrain.PIDConstants.Drive.kP,
            Constants.Drivetrain.PIDConstants.Drive.kI, Constants.Drivetrain.PIDConstants.Drive.kD,
            new TrapezoidProfile.Constraints(Constants.Drivetrain.MAX_ANGULAR_VELOCITY,
                    Constants.Drivetrain.MAX_ANGULAR_ACCELERATION));

    public SwerveModule(String name, int driveMotorChannel, int turnMotorChannel, int turnEncoderChannel,
            boolean driveMotorInverted, boolean turnEncoderInverted) {
        driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
        driveMotor.setInverted(driveMotorInverted);
        driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        turnMotor = new CANSparkMax(turnMotorChannel, MotorType.kBrushless);
        turnMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(2 * Math.PI * Constants.Drivetrain.WHEEL_RADIUS); // in meters
        driveEncoder.setVelocityConversionFactor((2 * Math.PI * Constants.Drivetrain.WHEEL_RADIUS) / 60.0);

        turnEncoder = new CANCoder(turnEncoderChannel);
        // there is an issue with absolute position vs position in CANCoders, namely
        // that the abs pos is sent a lot less frequently than position (every 100ms vs
        // every 10ms). according to this post:
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

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(turnEncoder.getPosition()));
    }

    public void setState(SwerveModuleState state) {
        double driveOutput = drivePIDController.calculate(driveEncoder.getVelocity(), state.speedMetersPerSecond);
        double turnOutput = turnPIDController.calculate(turnEncoder.getPosition(), state.angle.getRadians());

        driveMotor.set(driveOutput);
        turnMotor.set(turnOutput);
    }

    public void resetDriveEncoder() {
        driveEncoder.setPosition(0);
    }

    public double getDriveEncoderPosition() {
        return driveEncoder.getPosition();
    }

    public double getAngle() {
        return Math.toDegrees(turnEncoder.getPosition());
    }
}