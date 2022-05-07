package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.houndutil.logging.LogGroup;
import frc.houndutil.logging.LogProfileBuilder;
import frc.houndutil.logging.Logger;
import frc.robot.Constants;

/**
 * Climber subsystem, includes the climber arms, locks, and 2nd stage arms.
 * 
 * @author dr
 */
public class Climber extends SubsystemBase {
    /**
     * The primary climber motor.
     */
    private CANSparkMax primaryMotor = new CANSparkMax(Constants.Climber.CANIDs.PRIMARY, MotorType.kBrushless);

    /**
     * The secondary climber motor.
     */
    private CANSparkMax secondaryMotor = new CANSparkMax(Constants.Climber.CANIDs.SECONDARY, MotorType.kBrushless);

    /**
     * The double solenoid for the second stage of the climber
     */
    private DoubleSolenoid climberSecondStage = new DoubleSolenoid(PneumaticsModuleType.REVPH,
            Constants.Climber.Solenoids.CLIMBER_2ND_STAGE_CHANNEL_1,
            Constants.Climber.Solenoids.CLIMBER_2ND_STAGE_CHANNEL_2);

    /**
     * The lock for the climber.
     */
    private DoubleSolenoid climberLocks = new DoubleSolenoid(PneumaticsModuleType.REVPH,
            Constants.Climber.Solenoids.CLIMBER_LOCK_CHANNEL_1,
            Constants.Climber.Solenoids.CLIMBER_LOCK_CHANNEL_2);

    /**
     * The motor controller group containing both of the climber motors.
     */
    private MotorControllerGroup climberMotors = new MotorControllerGroup(primaryMotor, secondaryMotor);

    private LogGroup logger = new LogGroup("Climber",
            new Logger<?>[] {
                    new Logger<CANSparkMax>(primaryMotor, "Primary Motor",
                            LogProfileBuilder.buildCANSparkMaxLogItems(primaryMotor)),
                    new Logger<CANSparkMax>(secondaryMotor, "Secondary Motor",
                            LogProfileBuilder.buildCANSparkMaxLogItems(secondaryMotor)),
                    new Logger<DoubleSolenoid>(climberSecondStage, "Second Stage",
                            LogProfileBuilder.buildDoubleSolenoidLogItems(climberSecondStage)),
                    new Logger<DoubleSolenoid>(climberLocks, "Locks",
                            LogProfileBuilder.buildDoubleSolenoidLogItems(climberLocks))
            });

    /**
     * The constructor for the class. Sets the climber motors to the inverted status
     * described in Constants.java.
     */
    public Climber() {
        climberMotors.setInverted(Constants.Climber.IS_INVERTED);
    }

    @Override
    public void periodic() {
        logger.run();
    }

    /**
     * Gets the current encoder value of the climber
     * 
     * @return current distance traveled since last reset
     */
    public double getPosition() {
        return primaryMotor.getEncoder().getPosition();
    }

    /**
     * Sets the speed of the climber
     * 
     * @param speed speed from -1 to 1 at which to run the motors.
     */
    public void setSpeed(double speed) {
        climberMotors.set(speed);
    }

    /**
     * Resets the encoders by setting their positions to zero.
     */
    public void resetEncoders() {
        primaryMotor.getEncoder().setPosition(0);
    }

    /**
     * Extends the climber locking pneumatic.
     */
    public void extendLock() {
        climberLocks.set(Value.kReverse);
    }

    /**
     * Retracts the climber locking pneumatic.
     */
    public void retractLock() {
        climberLocks.set(Value.kForward);
    }

    /**
     * Get the state of the climber locks.
     * 
     * @return true if the locks are extended, false if not
     */
    public boolean getLock() {
        return climberLocks.get() == Value.kReverse;
    }

    /**
     * Extends the 2nd stage of the climber.
     */
    public void extendSecondStage() {
        climberSecondStage.set(Value.kForward);
    }

    /**
     * Retracts the 2nd stage of the climber.
     */
    public void retractSecondStage() {
        climberSecondStage.set(Value.kReverse);
    }
}
