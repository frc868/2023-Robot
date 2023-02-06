package frc.robot.subsystems;

import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.techhounds.houndutil.houndlib.Utils;
import com.techhounds.houndutil.houndlog.LogGroup;
import com.techhounds.houndutil.houndlog.LogProfileBuilder;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.loggers.DeviceLogger;
import com.techhounds.houndutil.houndlog.loggers.Logger;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;
import frc.robot.GamePieceLocation.GamePiece;
import frc.robot.GamePieceLocation.Level;

/**
 * The elevator subsystem, with two motors and motion profling.
 * 
 * @author hr
 * @author jt
 */

public class Elevator extends ProfiledPIDSubsystem {
    public static enum ElevatorPosition {
        BOTTOM(0),
        CONE_LOW(0),
        CONE_MID(0),
        CONE_HIGH(0),
        CUBE_LOW(0),
        CUBE_MID(0),
        CUBE_HIGH(0),
        HUMAN_PLAYER(0),
        TOP(0);

        public final double value;

        private ElevatorPosition(double value) {
            this.value = value;
        }
    }

    /** The left motor of the elevator. */
    private CANSparkMax leftMotor = new CANSparkMax(Constants.Elevator.CANIDs.LEFT_MOTOR, MotorType.kBrushless);

    /** The right motor of the elevator. */
    private CANSparkMax rightMotor = new CANSparkMax(Constants.Elevator.CANIDs.RIGHT_MOTOR, MotorType.kBrushless);

    /** The object that controlls both elevator motors. */
    private MotorControllerGroup motors = new MotorControllerGroup(leftMotor, rightMotor);

    /** Hall effect sensor that represents the minimum extension of the elevator. */
    private DigitalInput bottomHallEffect = new DigitalInput(Constants.Elevator.BOTTOM_HALL_EFFECT_PORT);

    /** Hall effect sensor that represents the maximum extension of the elevator. */
    private DigitalInput topHallEffect = new DigitalInput(Constants.Elevator.TOP_HALL_EFFECT_PORT);

    /**
     * The feedforward controller (calculates voltage based on the setpoint's
     * velocity and acceleration).
     */
    private ElevatorFeedforward feedforwardController = new ElevatorFeedforward(
            Constants.Elevator.Gains.kS,
            Constants.Elevator.Gains.kG,
            Constants.Elevator.Gains.kV,
            Constants.Elevator.Gains.kA);

    /**
     * Used to make sure that the PID controller does not enable unless the elevator
     * has been zeroed off of the hall effects.
     */
    private boolean isInitialized = false;

    /** The ligament of the complete mechanism body that this subsystem controls. */
    private MechanismLigament2d ligament;

    /**
     * Initializes the elevator.
     */
    public Elevator(MechanismLigament2d ligament) {
        super(new ProfiledPIDController(
                Constants.Elevator.Gains.kP.get(),
                Constants.Elevator.Gains.kI.get(),
                Constants.Elevator.Gains.kD.get(),
                new TrapezoidProfile.Constraints(
                        Constants.Elevator.MAX_VELOCITY_METERS_PER_SECOND.get(),
                        Constants.Elevator.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED.get())));

        getController().setTolerance(Constants.Elevator.Gains.TOLERANCE.get());

        Constants.Elevator.Gains.kP.setConsumer((d) -> getController().setP(d));
        Constants.Elevator.Gains.kI.setConsumer((d) -> getController().setI(d));
        Constants.Elevator.Gains.kD.setConsumer((d) -> getController().setD(d));
        Constants.Elevator.Gains.TOLERANCE.setConsumer((d) -> getController().setTolerance(d));
        Constants.Elevator.MAX_VELOCITY_METERS_PER_SECOND.setConsumer((d) -> getController()
                .setConstraints(
                        new TrapezoidProfile.Constraints(
                                d,
                                Constants.Elevator.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED.get())));

        Constants.Elevator.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED.setConsumer((d) -> getController()
                .setConstraints(
                        new TrapezoidProfile.Constraints(
                                Constants.Elevator.MAX_VELOCITY_METERS_PER_SECOND.get(),
                                d)));

        this.ligament = ligament;

        LoggingManager.getInstance().addGroup("Elevator", new LogGroup(
                new Logger[] {
                        new DeviceLogger<CANSparkMax>(leftMotor, "Primary Elevator Motor",
                                LogProfileBuilder.buildCANSparkMaxLogItems(leftMotor)),
                        new DeviceLogger<CANSparkMax>(rightMotor, "Secondary Elevator Motor",
                                LogProfileBuilder.buildCANSparkMaxLogItems(rightMotor))
                }));
    }

    /**
     * Runs every 20ms. This updates the ligament sent over NetworkTables.
     */
    @Override
    public void periodic() {
        super.periodic();
        ligament.setLength(-0.71 + getMeasurement());
    }

    /**
     * Uses the output from the ProfiledPIDController. Adds the output to the result
     * of the calculation from the feedforward controller.
     * 
     * @param output   the output of the ProfiledPIDController
     * @param setpoint the setpoint state of the ProfiledPIDController, for
     *                 feedforward
     */
    @Override
    public void useOutput(double output, TrapezoidProfile.State setpoint) {
        double feedforward = feedforwardController.calculate(setpoint.position, setpoint.velocity);
        leftMotor.setVoltage(output + feedforward);
        rightMotor.setVoltage(output + feedforward);
    }

    /**
     * Returns the measurement of the process variable used by the
     * ProfiledPIDController.
     * 
     * @return the output of the left motor's encoder
     */
    @Override
    public double getMeasurement() {
        return (leftMotor.getEncoder().getPosition() + rightMotor.getEncoder().getPosition()) / 2.0;
    }

    /**
     * Get the goal position of the elevator.
     * 
     * @return the goal position of the elevator
     */
    public double getGoal() {
        return getController().getGoal().position;
    }

    /**
     * Checks if the elevator is at its goal position.
     * 
     * @return true if the elevator is at its goal position
     */
    public boolean isAtGoal() {
        return getController().atGoal();
    }

    /**
     * Check if the elevator is lowered. This is useful for the subsystem safeties.
     * 
     * @return true of the elevator is at the bottom state and is within the
     *         tolerance of the controller.
     */
    public boolean isSafeForIntake() {
        return getMeasurement() - ElevatorPosition.BOTTOM.value < Constants.Elevator.Gains.TOLERANCE.get();
    }

    /**
     * Sets the value of each encoder to the minimum distance.
     */
    public void resetEncoders() {
        leftMotor.getEncoder().setPosition(0);
        rightMotor.getEncoder().setPosition(0);
    }

    /**
     * Stops both motors.
     */
    public void stop() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }

    /**
     * Sets the speed of the motors. Will refuse to go past the bottom or top points
     * of the elevator.
     * 
     * @param speed the speed, from -1.0 to 1.0
     */
    public void setSpeed(double speed) {
        if (!Utils.limitMechanism(bottomHallEffect.get(), topHallEffect.get(), speed))
            motors.set(speed);
    }

    /**
     * Sets the voltage of the motors. Will refuse to go past the bottom or top
     * points of the elevator.
     * 
     * @param speed the speed, from -1.0 to 1.0
     */
    public void setVoltage(double voltage) {
        if (!Utils.limitMechanism(bottomHallEffect.get(), topHallEffect.get(), voltage))
            motors.setVoltage(voltage);
    }

    /**
     * Creates an InstantCommand that sets the goal position of the elbow, and
     * enables the controller.
     * 
     * @param position an ElevatorPosition to set the elbow to
     * @return the command
     */
    public CommandBase setDesiredPositionCommand(ElevatorPosition position, Intake intake, LEDs leds) {
        return Commands.either(
                Commands.sequence(
                        runOnce(() -> setGoal(position.value)),
                        runOnce(this::enable),
                        Commands.waitUntil(this::isAtGoal)).finallyDo((d) -> this.disable()),
                leds.errorCommand(),
                () -> isInitialized);
    }

    public CommandBase setScoringPositionCommand(Supplier<GamePiece> gamePieceMode,
            Supplier<Level> scoringMode, Intake intake, LEDs leds) {
        return Commands.select(
                Map.of(
                        GamePiece.CONE,
                        Commands.select(
                                Map.of(
                                        Level.LOW,
                                        setDesiredPositionCommand(ElevatorPosition.CONE_LOW, intake, leds),
                                        Level.MIDDLE,
                                        setDesiredPositionCommand(ElevatorPosition.CONE_MID, intake, leds),
                                        Level.HIGH,
                                        setDesiredPositionCommand(ElevatorPosition.CONE_HIGH, intake, leds)),
                                scoringMode::get),
                        GamePiece.CUBE,
                        Commands.select(
                                Map.of(
                                        Level.LOW,
                                        setDesiredPositionCommand(ElevatorPosition.CUBE_LOW, intake, leds),
                                        Level.MIDDLE,
                                        setDesiredPositionCommand(ElevatorPosition.CUBE_MID, intake, leds),
                                        Level.HIGH,
                                        setDesiredPositionCommand(ElevatorPosition.CUBE_HIGH, intake, leds)),
                                scoringMode::get)),
                gamePieceMode::get);
    }

    /**
     * Creates a psuedo-StartEndCommand that runs the motors down at 10% speed until
     * the bottom hall effect sensor is reached, then zeros the encoders.
     */
    public CommandBase zeroEncoderCommand() {
        return runEnd(() -> setSpeed(-0.1), () -> {
            this.resetEncoders();
            isInitialized = true;
        }).until(bottomHallEffect::get);
    }

    public CommandBase setOverridenElevatorSpeedCommand(DoubleSupplier speed, Intake intake, LEDs leds) {
        return Commands.either(runOnce(() -> setSpeed(speed.getAsDouble())), leds.errorCommand(),
                intake::isSafeForElevator);
    }
}
