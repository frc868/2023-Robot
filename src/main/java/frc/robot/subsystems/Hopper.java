package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.houndutil.logging.LogGroup;
import frc.houndutil.logging.LogProfileBuilder;
import frc.houndutil.logging.Logger;
import frc.robot.Constants;

/**
 * The hopper subsystem, includes the hopper motors and the gatekeepers.
 * 
 * @author dr
 */
public class Hopper extends SubsystemBase {
    private CANSparkMax motor = new CANSparkMax(Constants.Hopper.CANIDs.MOTOR, MotorType.kBrushless);;
    private DoubleSolenoid gatekeepers = new DoubleSolenoid(PneumaticsModuleType.REVPH,
            Constants.Hopper.Solenoids.GATEKEEPER_CHANNEL_2,
            Constants.Hopper.Solenoids.GATEKEEPER_CHANNEL_1);

    private LogGroup logger = new LogGroup("Hopper",
            new Logger<?>[] {
                    new Logger<CANSparkMax>(motor, "Motor",
                            LogProfileBuilder.buildCANSparkMaxLogItems(motor)),
                    new Logger<DoubleSolenoid>(gatekeepers, "Gatekeepers",
                            LogProfileBuilder.buildDoubleSolenoidLogItems(gatekeepers))
            });

    /**
     * Constructs the hopper object.
     */
    public Hopper() {
        motor.setInverted(Constants.Hopper.IS_INVERTED);
    }

    @Override
    public void periodic() {
        logger.run();
    }

    /**
     * Sets the gatekeepers to the "in" position (non-blocking).
     */
    public void gatekeepersIn() {
        gatekeepers.set(DoubleSolenoid.Value.kForward);
    }

    /**
     * Sets the gatekeepers to the "out" position (blocking).
     */
    public void gatekeepersOut() {
        gatekeepers.set(DoubleSolenoid.Value.kReverse);
    }

    /**
     * Toggles the gatekeepers.
     */
    public void toggleGatekeepers() {
        gatekeepers.toggle();
    }

    /**
     * Runs the hopper motor at full power.
     */
    public void runMotor() {
        motor.set(1);
    }

    /**
     * Runs the hopper motor in reverse at full power.
     */
    public void reverseMotor() {
        motor.set(-1);
    }

    /**
     * Stops the motor.
     */
    public void stopMotor() {
        motor.set(0);
    }
}
