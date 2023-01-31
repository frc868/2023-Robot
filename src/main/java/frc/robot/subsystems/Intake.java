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
 * and intake solenoid
 * 
 * @author BAM
 */

public class Intake extends SubsystemBase {
    /**
     * Creates a new can spark motor controller for the right passover motor
     */
    private CANSparkMax rightPassoverMotor = new CANSparkMax(Constants.Intake.RIGHT_PASSOVER_MOTOR,
            MotorType.kBrushless);
    /**
     * Creates a new can spark motor controller for the left passover motor
     */
    private CANSparkMax leftPassoverMotor = new CANSparkMax(Constants.Intake.LEFT_PASSOVER_MOTOR,
            MotorType.kBrushless);
    /**
     * Creates new Double Solenoid object for the passover wheels
     */
    private DoubleSolenoid passoverSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
            6, 7); // untested
    /**
     * Creates new Double Solenoid object for the intake
     */
    private DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
            4, 5); // untested
    /**
     * Creates a motor controller group to allow both passovers to move at once
     */
    MotorControllerGroup passoverMotorController = new MotorControllerGroup(rightPassoverMotor,
            leftPassoverMotor);
    /**
     * Creates a new Compressor object
     */
    private PneumaticHub pneumaticHub = new PneumaticHub();

    /**
     * Intake constructor
     */
    public Intake() {
        rightPassoverMotor.setInverted(true);
        pneumaticHub.enableCompressorDigital();
        LoggingManager.getInstance().addGroup("Intake", new LogGroup(
                new Logger[] {
                        new DeviceLogger<CANSparkMax>(rightPassoverMotor, "Right Passover Motor",
                                LogProfileBuilder.buildCANSparkMaxLogItems(rightPassoverMotor)),
                        new DeviceLogger<CANSparkMax>(leftPassoverMotor, "Left Passover Motor",
                                LogProfileBuilder.buildCANSparkMaxLogItems(leftPassoverMotor)),
                        new DeviceLogger<DoubleSolenoid>(intakeSolenoid, "Intake Solenoid",
                                LogProfileBuilder.buildDoubleSolenoidLogItems(intakeSolenoid)),
                        new DeviceLogger<DoubleSolenoid>(passoverSolenoid, "Passover Solenoid",
                                LogProfileBuilder.buildDoubleSolenoidLogItems(passoverSolenoid)),
                        new DeviceLogger<PneumaticHub>(pneumaticHub, "Pneumatic Hub",
                                LogProfileBuilder.buildPneumaticHubLogItems(pneumaticHub))

                }));
    }

    /**
     * Sets Passover wheels to the down position
     */
    public void setPassoverDown() {
        passoverSolenoid.set(Value.kForward); // untested
    }

    /**
     * Sets Passover wheels to the up position
     */
    public void setPassoverUp() {
        passoverSolenoid.set(Value.kReverse); // untested
    }

    /**
     * Sets the intake to the down position
     */
    public void setIntakeDown() {
        intakeSolenoid.set(Value.kForward); // untested
    }

    /**
     * Sets the intake to the up position
     */
    public void setIntakeUp() {
        intakeSolenoid.set(Value.kReverse); // untested
    }

    /**
     * Runs the passover motors
     */
    public void runPassoverMotors() {
        passoverMotorController.set(.5);
    }

    /**
     * Stops the passover motors
     */
    public void stopPassoverMotors() {
        passoverMotorController.set(0);
    }
}
