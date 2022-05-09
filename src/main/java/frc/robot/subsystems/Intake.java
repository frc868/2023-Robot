package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.houndutil.houndlog.LogGroup;
import frc.houndutil.houndlog.LogProfileBuilder;
import frc.houndutil.houndlog.LoggingManager;
import frc.houndutil.houndlog.loggers.DeviceLogger;
import frc.houndutil.houndlog.loggers.Logger;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private CANSparkMax motor = new CANSparkMax(Constants.Intake.CANIDs.MOTOR,
            MotorType.kBrushless);
    private DoubleSolenoid solenoid = new DoubleSolenoid(
            PneumaticsModuleType.REVPH,
            Constants.Intake.Solenoids.INTAKE_CHANNEL_1,
            Constants.Intake.Solenoids.INTAKE_CHANNEL_2);

    public Intake() {
        motor.setInverted(Constants.Intake.IS_INVERTED);
        LoggingManager.getInstance().addGroup("Intake", new LogGroup(
                new Logger[] {
                        new DeviceLogger<CANSparkMax>(motor, "Motor",
                                LogProfileBuilder.buildCANSparkMaxLogItems(motor)),
                        new DeviceLogger<DoubleSolenoid>(solenoid, "Gatekeepers",
                                LogProfileBuilder.buildDoubleSolenoidLogItems(solenoid)),
                }));
    }

    /**
     * Sets the intake to the up position.
     */
    public void setUp() {
        solenoid.set(DoubleSolenoid.Value.kForward);
    }

    /**
     * Sets the intake to the down position.
     */
    public void setDown() {
        solenoid.set(DoubleSolenoid.Value.kReverse);
    }

    /**
     * Runs the intake motors.
     */
    public void runMotors() {
        motor.set(1);
    }

    /**
     * Runs the intake motors in reverse.
     */
    public void reverseMotors() {
        motor.set(-1);
    }

    /**
     * Stops the intake motors.
     */
    public void stop() {
        motor.set(0);
    }
}
