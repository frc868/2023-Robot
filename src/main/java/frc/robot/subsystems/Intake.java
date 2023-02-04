package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.techhounds.houndutil.houndlog.LogGroup;
import com.techhounds.houndutil.houndlog.LogProfileBuilder;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.loggers.Logger;
import com.techhounds.houndutil.houndlog.loggers.DeviceLogger;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Intake subsystem, contains two passover motor controllers, passover solenoid,
 * and intake solenoid.
 * 
 * @author bam
 */

public class Intake extends SubsystemBase {
    /**
     * Motors that drive the left passover.
     */
    private CANSparkMax leftPassoverMotor = new CANSparkMax(Constants.Intake.CANIDs.LEFT_PASSOVER_MOTOR,
            MotorType.kBrushless);
    /**
     * Motors that drive the right passover.
     */
    private CANSparkMax rightPassoverMotor = new CANSparkMax(Constants.Intake.CANIDs.RIGHT_PASSOVER_MOTOR,
            MotorType.kBrushless);
    /**
     * Solenoid that extends and retracts passover wheels.
     */
    private DoubleSolenoid passoverSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
            6, 7); // untested
    /**
     * Solenoid that extends and retracts the intake system.
     */
    private DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
            4, 5); // untested
    /**
     * Motor controller group for the passover motors.
     */
    MotorControllerGroup passoverMotors = new MotorControllerGroup(leftPassoverMotor,
            rightPassoverMotor);
    /**
     * Pnuematic Hub will become obsolete after rev updates their vendor rep.
     */
    private PneumaticHub pneumaticHub = new PneumaticHub();

    /**
     * Constructs the intake system.
     */
    public Intake() {
        rightPassoverMotor.setInverted(true);
        pneumaticHub.enableCompressorDigital();
        LoggingManager.getInstance().addGroup("Intake", new LogGroup(
                new Logger[] {
                        new DeviceLogger<CANSparkMax>(leftPassoverMotor, "Left Passover Motor",
                                LogProfileBuilder.buildCANSparkMaxLogItems(leftPassoverMotor)),
                        new DeviceLogger<CANSparkMax>(rightPassoverMotor, "Right Passover Motor",
                                LogProfileBuilder.buildCANSparkMaxLogItems(rightPassoverMotor)),
                        new DeviceLogger<DoubleSolenoid>(intakeSolenoid, "Intake Solenoid",
                                LogProfileBuilder.buildDoubleSolenoidLogItems(intakeSolenoid)),
                        new DeviceLogger<DoubleSolenoid>(passoverSolenoid, "Passover Solenoid",
                                LogProfileBuilder.buildDoubleSolenoidLogItems(passoverSolenoid)),
                        new DeviceLogger<PneumaticHub>(pneumaticHub, "Pneumatic Hub",
                                LogProfileBuilder.buildPneumaticHubLogItems(pneumaticHub))

                }));
    }

    /**
     * Sets Passover wheels to the down position.
     */
    public void setPassoverExtended() {
        passoverSolenoid.set(Value.kForward); // untested
    }

    /**
     * Sets Passover wheels to the up position.
     */
    public void setPassoverRetracted() {
        passoverSolenoid.set(Value.kReverse); // untested
    }

    /**
     * Sets the intake to the down position.
     */
    public void setIntakeDown() {
        intakeSolenoid.set(Value.kForward); // untested
    }

    /**
     * Sets the intake to the up position.
     */
    public void setIntakeUp() {
        intakeSolenoid.set(Value.kReverse); // untested
    }

    /**
     * Runs the passover motors.
     */
    public void runPassoverMotors() {
        passoverMotors.set(.5);
    }

    /**
     * Stops the passover motors.
     */
    public void stopPassoverMotors() {
        passoverMotors.set(0);
    }
}
