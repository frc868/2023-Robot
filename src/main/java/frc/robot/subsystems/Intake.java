package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.techhounds.houndutil.houndlog.LogGroup;
import com.techhounds.houndutil.houndlog.LogProfileBuilder;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.loggers.DeviceLogger;
import com.techhounds.houndutil.houndlog.loggers.Logger;
import com.techhounds.houndutil.houndlog.logitems.BooleanLogItem;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
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

    /** The ligament of the complete mechanism body that this subsystem controls. */
    private MechanismLigament2d ligament;

    /** The beam break that detects if a game piece is in the robot. */
    private DigitalInput gamePieceDetector = new DigitalInput(Constants.Intake.GAME_PIECE_SENSOR_PORT);

    /**
     * The wrapper of the gamePieceDetector for simulation. This allows you to set
     * the values of the sensor in sim without changing the values IRL. This value
     * is not used when the robot is running IRL.
     */
    private DIOSim gamePieceDetectorSim;

    /**
     * Initializes the intake system.
     */
    public Intake(MechanismLigament2d ligament) {
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
                        new BooleanLogItem("Is Game Piece Detected", this::isGamePieceDetected),
                        new BooleanLogItem("Is Safe for Elevator", this::isSafeForElevator)
                }));
        this.ligament = ligament;

        if (RobotBase.isSimulation()) {
            gamePieceDetectorSim = new DIOSim(gamePieceDetector);
            gamePieceDetectorSim.setValue(false);
        }
    }

    /**
     * Runs every 20ms. This updates the ligament sent over NetworkTables.
     */
    @Override
    public void periodic() {
        super.periodic();
        this.ligament.setAngle(intakeSolenoid.get() == Value.kForward ? 0 : 110);
    }

    /**
     * Checks if the it is safe fo the elevator to move based off of the intake:
     * 1. the passover is retracted
     * 2. the intake is up
     * 
     * @return true if safe to move
     */
    public boolean isSafeForElevator() {
        return passoverSolenoid.get() == Value.kReverse && intakeSolenoid.get() == Value.kReverse;
    }

    /**
     * Checks if the intake is safe to move based on the elevator.
     * 
     * @param elevator
     * @return true if safe to move
     */
    private boolean isSafeToMove(Elevator elevator) {
        return elevator.isSafeForIntake();
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
                () -> isSafeToMove(elevator)).withName("Set Passover Extended");
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
        return runOnce(() -> passoverSolenoid.set(Value.kReverse)).withName("Set Passover Retracted");
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
                        () -> isSafeToMove(elevator))
                .withName("Set Intake Down"); // untested
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
                        () -> isSafeToMove(elevator))
                .withName("Set Intake Up"); // untested
    }

    /**
     * Creates a StartEndCommand (requiring this subsystem) to run the passover
     * motors.
     * This will run the motors until the command is interrupted/cancelled.
     * 
     * @return the command
     */
    public CommandBase runPassoverMotorsCommand() {
        return startEnd(
                () -> passoverMotors.setVoltage(6),
                () -> passoverMotors.setVoltage(0))
                .withName("Run Passover Motors");
    }

    /**
     * Check if a game piece is detected to be ready to be pinced.
     * 
     * @return true if a game piece is detected inside the robot
     */
    public boolean isGamePieceDetected() {
        return gamePieceDetector.get();
    }

    /**
     * Creates a command that toggles on the game piece detector to simulate that a
     * game piece has entered the robot. This will only work in sim.
     * 
     * @return the command
     */
    public CommandBase simGamePieceDetectedCommand() {
        return Commands.runOnce(() -> gamePieceDetectorSim.setValue(true))
                .andThen(Commands.waitSeconds(1))
                .andThen(() -> gamePieceDetectorSim.setValue(false)).withName("Sim Game Piece Detected");
    }
}
