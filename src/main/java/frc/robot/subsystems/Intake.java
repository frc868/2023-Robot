package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.techhounds.houndutil.houndlib.SparkMaxConfigurator;
import com.techhounds.houndutil.houndlog.LogGroup;
import com.techhounds.houndutil.houndlog.LogProfileBuilder;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.enums.LogLevel;
import com.techhounds.houndutil.houndlog.loggers.DeviceLogger;
import com.techhounds.houndutil.houndlog.loggers.Logger;
import com.techhounds.houndutil.houndlog.logitems.BooleanLogItem;

import edu.wpi.first.math.Pair;
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
    private CANSparkMax leftPassoverMotor = new CANSparkMax(Constants.CAN.PASSOVER_LEFT_MOTOR,
            MotorType.kBrushless);
    /**
     * The motor that drives the right side of the passover.
     */
    private CANSparkMax rightPassoverMotor = new CANSparkMax(Constants.CAN.PASSOVER_RIGHT_MOTOR,
            MotorType.kBrushless);
    /**
     * The solenoid that controls the passover extending or retracting.
     */
    private DoubleSolenoid passoverSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
            Constants.Pneumatics.PASSOVER[0], Constants.Pneumatics.PASSOVER[1]);
    /**
     * The solenoid that has shared control over the cubapult and ramrods.
     */
    private DoubleSolenoid cubapultRamrodSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
            Constants.Pneumatics.CUBAPULT[0], Constants.Pneumatics.CUBAPULT[1]);
    /**
     * The object that controlls both passover motors.
     */
    MotorControllerGroup passoverMotors = new MotorControllerGroup(leftPassoverMotor, rightPassoverMotor);

    /** The ligament of the complete mechanism body that this subsystem controls. */
    private MechanismLigament2d ligament;

    /** The beam break that detects if a game piece is in the robot. */
    private DigitalInput gamePieceDetector = new DigitalInput(Constants.DIO.GAME_PIECE_SENSOR);

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
        SparkMaxConfigurator.configure(leftPassoverMotor)
                .withCurrentLimit(25)
                .withInverted(true)
                .burnFlash();

        SparkMaxConfigurator.configure(rightPassoverMotor)
                .withCurrentLimit(25)
                .burnFlash();

        LoggingManager.getInstance().addGroup("Intake", new LogGroup(
                new Logger[] {
                        new DeviceLogger<CANSparkMax>(leftPassoverMotor, "Left Passover Motor",
                                LogProfileBuilder.buildCANSparkMaxLogItems(leftPassoverMotor)),
                        new DeviceLogger<CANSparkMax>(rightPassoverMotor, "Right Passover Motor",
                                LogProfileBuilder.buildCANSparkMaxLogItems(rightPassoverMotor)),
                        new DeviceLogger<DoubleSolenoid>(cubapultRamrodSolenoid, "Cubapult Solenoid",
                                LogProfileBuilder.buildDoubleSolenoidLogItems(cubapultRamrodSolenoid)),
                        new DeviceLogger<DoubleSolenoid>(passoverSolenoid, "Passover Solenoid",
                                LogProfileBuilder.buildDoubleSolenoidLogItems(passoverSolenoid)),
                        new BooleanLogItem("Is Game Piece Detected", this::isGamePieceDetected, LogLevel.MAIN),
                }));

        this.ligament = ligament;

        if (RobotBase.isSimulation()) {
            gamePieceDetectorSim = new DIOSim(gamePieceDetector);
            gamePieceDetectorSim.setValue(true);
        }
    }

    /**
     * Runs every 20ms. This updates the ligament sent over NetworkTables.
     */
    @Override
    public void periodic() {
        super.periodic();
        ligament.setAngle(cubapultRamrodSolenoid.get() == Value.kForward ? -90 : 0);
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
    public CommandBase setPassoversExtendedCommand(Elevator elevator) {
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
    public CommandBase setPassoversRetractedCommand(Elevator elevator) {
        return runOnce(() -> passoverSolenoid.set(Value.kForward)).withName("Set Passover Retracted");
    }

    /**
     * Creates an InstantCommand that sets the cubapult to the primed position.
     * 
     * @return the command
     */
    public CommandBase setCubapultPrimed() {
        return runOnce(() -> cubapultRamrodSolenoid.set(Value.kForward)).withName("Set Cubapult Primed"); // untested
    }

    /**
     * Creates an InstantCommand that sets the Cubapult to the released position.
     * 
     * @return the command
     */
    public CommandBase setCubapultReleased() {
        return runOnce(() -> cubapultRamrodSolenoid.set(Value.kReverse)).withName("Set Cubapult Released"); // untested
    }

    /**
     * Creates an InstantCommand that sets the ramrods to the extended position.
     * 
     * @return the command
     */
    public CommandBase setRamrodsExtended() {
        return runOnce(() -> cubapultRamrodSolenoid.set(Value.kReverse)).withName("Set Ramrods Extended"); // untested
    }

    /**
     * Creates an InstantCommand that sets the ramrods to the retracted position.
     * 
     * @return the command
     */
    public CommandBase setRamrodsRetracted() {
        return runOnce(() -> cubapultRamrodSolenoid.set(Value.kForward)).withName("Set Ramrods Released"); // untested
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
     * Creates a StartEndCommand (requiring this subsystem) to run the passover
     * motors.
     * This will run the motors until the command is interrupted/cancelled.
     * 
     * @return the command
     */
    public CommandBase startPassoverMotorsCommand() {
        return runOnce(() -> passoverMotors.setVoltage(6))
                .withName("Start Passover Motors");
    }

    /**
     * Creates a StartEndCommand (requiring this subsystem) to run the passover
     * motors.
     * This will run the motors until the command is interrupted/cancelled.
     * 
     * @return the command
     */
    public CommandBase stopPassoverMotorsCommand() {
        return runOnce(() -> passoverMotors.setVoltage(0))
                .withName("Stop Passover Motors");
    }

    /**
     * Creates a StartEndCommand (requiring this subsystem) to run the passover
     * motors in reverse.
     * This will run the motors until the command is interrupted/cancelled.
     * 
     * @return the command
     */
    public CommandBase reversePassoverMotorsCommand() {
        return startEnd(
                () -> passoverMotors.setVoltage(-6),
                () -> passoverMotors.setVoltage(0))
                .withName("Run Passover Motors");
    }

    /**
     * Check if a game piece is detected to be ready to be pinced.
     * 
     * @return true if a game piece is detected inside the robot
     */
    public boolean isGamePieceDetected() {
        return !gamePieceDetector.get();
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
