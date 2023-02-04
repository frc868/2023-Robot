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
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Intake subsystem, contains the motors that run the passovers, and the
 * pneumatics for the passover and intake.
 * 
 * @author bam
 */

public class Intake extends SubsystemBase {
    /**
     * The motor that drives the left side of the passover.
     */
    private CANSparkMax leftPassoverMotor = new CANSparkMax(Constants.Intake.CANIDs.LEFT_MOTOR,
            MotorType.kBrushless);
    /**
     * The motor that drives the right side of the passover.
     */
    private CANSparkMax rightPassoverMotor = new CANSparkMax(Constants.Intake.CANIDs.RIGHT_MOTOR,
            MotorType.kBrushless);
    /**
     * The solenoid that controls the passover extending or retracting.
     */
    private DoubleSolenoid passoverSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
            Constants.Intake.Pneumatics.PASSOVER[0], Constants.Intake.Pneumatics.PASSOVER[1]);
    /**
     * The solenoid that controls the intake moving up or down.
     */
    private DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
            Constants.Intake.Pneumatics.INTAKE[0], Constants.Intake.Pneumatics.INTAKE[1]);
    /**
     * The object that controlls both passover motors.
     */
    MotorControllerGroup passoverMotors = new MotorControllerGroup(leftPassoverMotor,
            rightPassoverMotor);

    /**
     * Initializes the intake system.
     */
    public Intake() {
        rightPassoverMotor.setInverted(true);
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
                }));
    }

    /**
     * Sets the passover to the extended position. This means that it is outside of
     * frame perimeter and is able to grip and index a game piece.
     */
    public void setPassoverExtended() {
        passoverSolenoid.set(Value.kForward); // untested
    }

    /**
     * Sets the passover to the retracted position. This means that it is retracted
     * into the robot and is not able to grip or index a game piece.
     */
    public void setPassoverRetracted() {
        passoverSolenoid.set(Value.kReverse); // untested
    }

    /**
     * Sets the intake to the down position. This means that it is outside of frame
     * perimeter and able to manipulate a game piece.
     */
    public void setIntakeDown() {
        intakeSolenoid.set(Value.kForward); // untested
    }

    /**
     * Sets the intake to the up position. This means that it is inside frame
     * perimeter.
     */
    public void setIntakeUp() {
        intakeSolenoid.set(Value.kReverse); // untested
    }

    /**
     * Runs the passover motors.
     */
    public void runPassoverMotors() {
        passoverMotors.set(.5); // untested
    }

    /**
     * Stops the passover motors.
     */
    public void stopPassoverMotors() {
        passoverMotors.set(0);
    }
}
