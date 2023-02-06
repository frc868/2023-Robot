package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.techhounds.houndutil.houndlog.LogGroup;
import com.techhounds.houndutil.houndlog.LogProfileBuilder;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.loggers.DeviceLogger;
import com.techhounds.houndutil.houndlog.loggers.Logger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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

    /** The beam break that detects if a game piece is in the robot. */
    private DigitalInput gamePieceDetector = new DigitalInput(Constants.Intake.GAME_PIECE_SENSOR_PORT);

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

    public boolean isSafeForElevator() {
        return passoverSolenoid.get() == Value.kReverse && intakeSolenoid.get() == Value.kReverse;
    }

    /**
     * Creates an InstantCommand that sets the passovers to the
     * extended position.
     * This means that it is outside of frame perimeter and able to grip and
     * index a game piece.
     * 
     * @return the command
     */
    public CommandBase setPassoverExtendedCommand(Elevator elevator, LEDs leds) {
        return Commands.either(
                runOnce(() -> passoverSolenoid.set(Value.kForward)),
                leds.errorCommand(),
                () -> elevator.isSafeForIntake());
    }

    /**
     * Creates an InstantCommand that sets the passover to the
     * retracted position.
     * 
     * This means that it is retracted into the robot and is not able to grip or
     * index a game piece.
     * 
     * @return the command
     */
    public CommandBase setPassoverRetractedCommand(Elevator elevator, LEDs leds) {
        return Commands
                .either(runOnce(() -> passoverSolenoid.set(Value.kReverse)), leds.errorCommand(),
                        elevator::isSafeForIntake); // untested
    }

    /**
     * Creates an InstantCommand that sets the intake to the up
     * position. This means that it is inside frame perimeter.
     * 
     * @return the command
     */
    public CommandBase setIntakeDownCommand(Elevator elevator, LEDs leds) {
        return Commands
                .either(runOnce(() -> intakeSolenoid.set(Value.kForward)), leds.errorCommand(),
                        elevator::isSafeForIntake); // untested
    }

    /**
     * Creates an InstantCommand that sets the intake to the up
     * position. This means that it is inside frame perimeter.
     * 
     * @return the command
     */
    public CommandBase setIntakeUpCommand(Elevator elevator, LEDs leds) {
        return Commands
                .either(runOnce(() -> intakeSolenoid.set(Value.kReverse)), leds.errorCommand(),
                        elevator::isSafeForIntake); // untested
    }

    /**
     * Creates a StartEndCommand (requiring this subsystem) to run the passover
     * motors.
     * This will run the motors until the command is interrupted/cancelled.
     * 
     * @return the command
     */
    public CommandBase runPassoverMotorsCommand() {
        return startEnd(() -> passoverMotors.set(.5), () -> passoverMotors.set(0));
    }

    /**
     * Check if a game piece is detected to be ready to be pinced.
     * 
     * @return true if a game piece is detected inside the robot
     */
    public boolean isGamePieceDetected() {
        return gamePieceDetector.get();
    }
}
