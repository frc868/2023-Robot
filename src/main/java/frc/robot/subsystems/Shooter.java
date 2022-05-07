package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.houndutil.logging.LogGroup;
import frc.houndutil.logging.LogProfileBuilder;
import frc.houndutil.logging.LogType;
import frc.houndutil.logging.Logger;
import frc.houndutil.logging.SingleItemLogger;
import frc.robot.Constants;

public class Shooter extends PIDSubsystem {
    private CANSparkMax primaryMotor = new CANSparkMax(Constants.Shooter.CANIDs.PRIMARY,
            MotorType.kBrushless);
    private CANSparkMax secondaryMotor = new CANSparkMax(Constants.Shooter.CANIDs.SECONDARY,
            MotorType.kBrushless);
    private MotorControllerGroup shooterMotors = new MotorControllerGroup(primaryMotor, secondaryMotor);

    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Shooter.kS, Constants.Shooter.kV);

    private LogGroup logger = new LogGroup("Shooter",
            new Logger<?>[] {
                    new Logger<CANSparkMax>(primaryMotor, "Primary Motor",
                            LogProfileBuilder.buildCANSparkMaxLogItems(primaryMotor)),
                    new Logger<CANSparkMax>(secondaryMotor, "Secondary Motor",
                            LogProfileBuilder.buildCANSparkMaxLogItems(secondaryMotor)),
                    new SingleItemLogger<Double>(LogType.NUMBER, "PID Setpoint", this::getSetpoint),
                    new SingleItemLogger<Double>(LogType.NUMBER, "PID Measurement", this::getMeasurement),
                    new SingleItemLogger<Double>(LogType.NUMBER, "Velocity", this::getVelocity)
            });

    public Shooter() {
        super(new PIDController(Constants.Shooter.kP, Constants.Shooter.kI, Constants.Shooter.kD));
        shooterMotors.setInverted(Constants.Shooter.IS_INVERTED);
    }

    @Override
    public void periodic() {
        logger.run();
    }

    @Override
    public double getMeasurement() {
        return getVelocity();
    }

    @Override
    public void useOutput(double output, double setpoint) {
        setSpeedVolts(output + feedforward.calculate(setpoint));
    }

    public void setSpeed(double speed) {
        shooterMotors.set(speed);
    }

    public void setSpeedVolts(double volts) {
        shooterMotors.setVoltage(volts);
    }

    /**
     * Resets the shooter
     */
    public void resetEncoders() {
        primaryMotor.getEncoder().setPosition(0);
    }

    /**
     * Gets the current velocity of the shooter
     * 
     * @return current speed of the shooter, in rpm
     */
    public double getVelocity() {
        return primaryMotor.getEncoder().getVelocity();
    }

    /**
     * Stops the shooter
     */
    public void stop() {
        setSpeed(0);
    }
}
