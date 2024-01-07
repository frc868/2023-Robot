package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.techhounds.houndutil.houndlib.SparkMaxConfigurator;
import com.techhounds.houndutil.houndlog.interfaces.Log;
import com.techhounds.houndutil.houndlog.interfaces.LoggedObject;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.robot.Constants.Intake.*;

/**
 * Intake subsystem, contains the motors that run the passovers, and the
 * pneumatics for the passover and intake.
 * 
 * @author bam
 */
@LoggedObject
public class Intake extends SubsystemBase {
    /**
     * The motor that drives the left side of the passover.
     */
    @Log
    private CANSparkMax leftPassoverMotor;

    /**
     * The motor that drives the right side of the passover.
     */
    @Log
    private CANSparkMax rightPassoverMotor;

    /**
     * The solenoid that controls the passover extending or retracting.
     */
    @Log
    private DoubleSolenoid passoverSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
            PASSOVER_PORTS[0], PASSOVER_PORTS[1]);

    /**
     * The solenoid that has shared control over the cubapult and ramrods.
     */
    @Log
    private DoubleSolenoid cubapultRamrodSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
            CUBAPULT_PORTS[0], CUBAPULT_PORTS[1]);

    /** The ligament of the complete mechanism body that this subsystem controls. */
    private MechanismLigament2d ligament;

    private Timer passoverPoseTimer = new Timer();

    /**
     * Initializes the intake system.
     */
    public Intake(MechanismLigament2d ligament) {
        leftPassoverMotor = SparkMaxConfigurator.create(
                PASSOVER_LEFT_MOTOR_ID, MotorType.kBrushless, false,
                (s) -> s.setIdleMode(IdleMode.kBrake),
                (s) -> s.setSmartCurrentLimit(PASSOVER_CURRENT_LIMIT));

        rightPassoverMotor = SparkMaxConfigurator.create(
                PASSOVER_RIGHT_MOTOR_ID, MotorType.kBrushless, true,
                (s) -> s.setIdleMode(IdleMode.kBrake),
                (s) -> s.setSmartCurrentLimit(PASSOVER_CURRENT_LIMIT),
                (s) -> s.follow(leftPassoverMotor, true));

        this.ligament = ligament;

        new Trigger(() -> passoverSolenoid.get() == Value.kForward)
                .onTrue(Commands.runOnce(passoverPoseTimer::restart));
        new Trigger(() -> passoverSolenoid.get() == Value.kReverse)
                .onTrue(Commands.runOnce(passoverPoseTimer::restart));
    }

    /**
     * Runs every 20ms. This updates the ligament sent over NetworkTables.
     */
    @Override
    public void periodic() {
        super.periodic();
        ligament.setAngle(cubapultRamrodSolenoid.get() == Value.kForward ? -90 : 0);
    }

    @Log
    public Pose3d getLeftPassoverComponentPose() {
        return LEFT_PASSOVER_RETRACTED_POSE.interpolate(
                LEFT_PASSOVER_EXTENDED_POSE,
                passoverSolenoid.get() == Value.kReverse
                        ? passoverPoseTimer.get() / PASSOVER_MOVEMENT_TIME
                        : 1 - passoverPoseTimer.get() / PASSOVER_MOVEMENT_TIME);
    }

    @Log
    public Pose3d getRightPassoverComponentPose() {
        return RIGHT_PASSOVER_RETRACTED_POSE.interpolate(
                RIGHT_PASSOVER_EXTENDED_POSE,
                passoverSolenoid.get() == Value.kReverse
                        ? passoverPoseTimer.get() / PASSOVER_MOVEMENT_TIME
                        : 1 - passoverPoseTimer.get() / PASSOVER_MOVEMENT_TIME);
    }

    /**
     * Checks if the it is safe for the elevator to move based off of the intake:
     * 1. the passover is retracted
     * 2. the intake is up
     * 
     * @return true if safe to move
     */
    public Pair<Boolean, String> isSafeForElevator() {
        boolean safe = true;
        String str = "none";

        // if (passoverSolenoid.get() == Value.kReverse) {
        // safe = false;
        // str = "Passovers not retracted: cannot move elevator";
        // }

        // if (intakeSolenoid.get() != Value.kReverse) {
        // safe = false;
        // str = "Intake not up: cannot move elevator";
        // }

        return new Pair<Boolean, String>(safe, str);
    }

    /**
     * Creates an InstantCommand that sets the passovers to the
     * extended position.
     * This means that it is outside of frame perimeter and able to grip and
     * index a game piece.
     * 
     * @return the command
     */
    public Command setPassoversExtendedCommand(Elevator elevator) {
        // return Commands.either(
        // runOnce(() -> passoverSolenoid.set(Value.kForward)),
        // leds.errorCommand(),
        // () -> isSafeToMove(elevator)).withName("Set Passover Extended");
        return runOnce(() -> passoverSolenoid.set(Value.kReverse)).withName("Set Passover Extended");
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
    public Command setPassoversRetractedCommand(Elevator elevator) {
        return runOnce(() -> passoverSolenoid.set(Value.kForward)).withName("Set Passover Retracted");
    }

    /**
     * Creates an InstantCommand that sets the cubapult to the primed position.
     * 
     * @return the command
     */
    public Command setCubapultPrimed() {
        return runOnce(() -> cubapultRamrodSolenoid.set(Value.kForward)).withName("Set Cubapult Primed"); // untested
    }

    /**
     * Creates an InstantCommand that sets the Cubapult to the released position.
     * 
     * @return the command
     */
    public Command setCubapultReleased() {
        return runOnce(() -> cubapultRamrodSolenoid.set(Value.kReverse)).withName("Set Cubapult Released"); // untested
    }

    /**
     * Creates an InstantCommand that sets the ramrods to the extended position.
     * 
     * @return the command
     */
    public Command setRamrodsExtendedCommand() {
        return Commands.runOnce(() -> cubapultRamrodSolenoid.set(Value.kReverse)).withName("Set Ramrods Extended"); // untested
    }

    /**
     * Creates an InstantCommand that sets the ramrods to the retracted position.
     * 
     * @return the command
     */
    public Command setRamrodsRetractedCommand() {
        return Commands.runOnce(() -> cubapultRamrodSolenoid.set(Value.kForward)).withName("Set Ramrods Released"); // untested
    }

    /**
     * Creates an InstantCommand that sets the ramrods to the retracted position.
     * 
     * @return the command
     */
    public Command toggleRamrodsCommand() {
        return Commands.startEnd(
                () -> cubapultRamrodSolenoid.set(Value.kForward),
                () -> cubapultRamrodSolenoid.set(Value.kReverse)).withName("Toggle Ramrods"); // untested
    }

    /**
     * Creates a StartEndCommand (requiring this subsystem) to run the passover
     * motors.
     * This will run the motors until the command is interrupted/cancelled.
     * 
     * @return the command
     */
    public Command runPassoverMotorsCommand() {
        return startEnd(
                () -> leftPassoverMotor.setVoltage(6),
                () -> leftPassoverMotor.setVoltage(0))
                .withName("Run Passover Motors");
    }

    /**
     * Creates a StartEndCommand (requiring this subsystem) to run the passover
     * motors.
     * This will run the motors until the command is interrupted/cancelled.
     * 
     * @return the command
     */
    public Command startPassoverMotorsCommand() {
        return runOnce(() -> leftPassoverMotor.setVoltage(6))
                .withName("Start Passover Motors");
    }

    /**
     * Creates a StartEndCommand (requiring this subsystem) to run the passover
     * motors.
     * This will run the motors until the command is interrupted/cancelled.
     * 
     * @return the command
     */
    public Command stopPassoverMotorsCommand() {
        return runOnce(() -> leftPassoverMotor.setVoltage(0))
                .withName("Stop Passover Motors");
    }

    /**
     * Creates a StartEndCommand (requiring this subsystem) to run the passover
     * motors in reverse.
     * This will run the motors until the command is interrupted/cancelled.
     * 
     * @return the command
     */
    public Command reversePassoverMotorsCommand() {
        return startEnd(
                () -> leftPassoverMotor.setVoltage(-6),
                () -> leftPassoverMotor.setVoltage(0))
                .withName("Run Passover Motors");
    }
}
