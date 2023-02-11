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
import com.techhounds.houndutil.houndlog.enums.LogLevel;
import com.techhounds.houndutil.houndlog.loggers.DeviceLogger;
import com.techhounds.houndutil.houndlog.loggers.Logger;
import com.techhounds.houndutil.houndlog.logitems.BooleanLogItem;
import com.techhounds.houndutil.houndlog.logitems.DoubleLogItem;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;
import frc.robot.Overrides;
import frc.robot.GamePieceLocation.GamePiece;
import frc.robot.GamePieceLocation.Level;
import frc.robot.commands.RobotStates;

/**
 * The elevator subsystem, with two motors and motion profling.
 * 
 * @author hr
 * @author jt
 */

public class Elevator extends ProfiledPIDSubsystem {
    public static enum ElevatorPosition {
        BOTTOM(0),
        CONE_LOW(Units.inchesToMeters(10)),
        CONE_MID(Units.inchesToMeters(30)),
        CONE_HIGH(Units.inchesToMeters(60)),
        CUBE_LOW(Units.inchesToMeters(10)),
        CUBE_MID(Units.inchesToMeters(30)),
        CUBE_HIGH(Units.inchesToMeters(60)),
        HUMAN_PLAYER(0),
        TOP(Units.inchesToMeters(80));

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

    /** The ligament of the complete mechanism body that this subsystem controls. */
    private MechanismLigament2d ligament;

    /**
     * The physics simulator for this mechansim. The elevator sim takes in voltages
     * from the motor and simulates an elevator under the influence of gravity.
     */
    private ElevatorSim elevatorSim = new ElevatorSim(
            DCMotor.getNEO(2),
            12,
            3.63,
            Units.inchesToMeters(2.4),
            Units.inchesToMeters(0),
            Units.inchesToMeters(80),
            true);

    /**
     * The wrapper of the bottomHallEffect for simulation. This allows you to set
     * the values of the sensor in sim without changing the values IRL. This value
     * is not used when the robot is running IRL.
     */
    private DIOSim bottomHallEffectSim;

    /**
     * The wrapper of the topHallEffect for simulation. This allows you to set
     * the values of the sensor in sim without changing the values IRL. This value
     * is not used when the robot is running IRL.
     */
    private DIOSim topHallEffectSim;

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

        rightMotor.follow(leftMotor, true);

        leftMotor.getEncoder().setPositionConversionFactor(Constants.Elevator.ENCODER_DISTANCE_TO_METERS);
        leftMotor.getEncoder().setVelocityConversionFactor(Constants.Elevator.ENCODER_DISTANCE_TO_METERS / 60.0);

        LoggingManager.getInstance().addGroup("Elevator", new LogGroup(
                new Logger[] {
                        new BooleanLogItem("Bottom Hall Effect", bottomHallEffect::get),
                        new BooleanLogItem("Top Hall Effect", topHallEffect::get),
                        new DoubleLogItem("Actual Position", () -> this.getMeasurement(), LogLevel.MAIN),
                        new DeviceLogger<CANSparkMax>(leftMotor, "Primary Elevator Motor",
                                LogProfileBuilder.buildCANSparkMaxLogItems(leftMotor)),
                        new DeviceLogger<CANSparkMax>(rightMotor, "Secondary Elevator Motor",
                                LogProfileBuilder.buildCANSparkMaxLogItems(rightMotor)),
                }));

        if (RobotBase.isSimulation()) {
            bottomHallEffectSim = new DIOSim(bottomHallEffect);
            bottomHallEffectSim.setValue(false);
            topHallEffectSim = new DIOSim(topHallEffect);
            topHallEffectSim.setValue(false);
            RobotStates.enableInitialized();
        }
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
     * Updates simulation-specific variables.
     */
    @Override
    public void simulationPeriodic() {
        // set the input (the voltage of the motor)
        elevatorSim.setInput(leftMotor.getAppliedOutput());
        // update the sim
        elevatorSim.update(0.020);
        leftMotor.getEncoder().setPosition(elevatorSim.getPositionMeters());
        bottomHallEffectSim.setValue(elevatorSim.getPositionMeters() <= 0);
        topHallEffectSim.setValue(elevatorSim.getPositionMeters() >= 2.03);
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
        double feedforward = feedforwardController.calculate(setpoint.velocity);
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
        return leftMotor.getEncoder().getPosition();
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
     * @return true if the elevator is at the bottom state and is within the
     *         tolerance of the controller.
     */
    public boolean isSafeForIntake() {
        return Math.abs(getMeasurement() - ElevatorPosition.BOTTOM.value) < Constants.Elevator.Gains.TOLERANCE.get();
    }

    /**
     * Check if the elevator isn't lowered. This is useful for the subsystem
     * safeties.
     * 
     * @return true if the elevator is NOT at the bottom state and is within the
     *         tolerance of the controller.
     */
    public boolean isSafeForWrist() {
        return !(Math.abs(getMeasurement() - ElevatorPosition.BOTTOM.value) < Constants.Elevator.Gains.TOLERANCE.get());
    }

    /**
     * Check if the elevator is safe to move.
     * 
     * @return true if the elevator is safe to move
     */
    private boolean isSafeToMove(Intake intake) {
        return RobotStates.isInitialized() && intake.isSafeForElevator();
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
        if (!Utils.limitMechanism(bottomHallEffect.get(), topHallEffect.get(), speed)) {
            if (RobotBase.isReal()) {
                motors.set(speed);
            } else {
                motors.setVoltage(speed * 12.0);
            }
        } else {
            motors.setVoltage(0);
        }

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
        else {
            motors.setVoltage(0);
        }
    }

    /**
     * Creates an InstantCommand that sets the goal position of the elevator, and
     * enables the controller.
     * 
     * @param position an ElevatorPosition to set the elevator to
     * @return the command
     */
    public CommandBase setDesiredPositionCommand(ElevatorPosition position, Intake intake, LEDs leds) {
        return Commands.either(
                Commands.sequence(
                        runOnce(() -> setGoal(position.value)),
                        runOnce(this::enable),
                        Commands.waitUntil(this::isAtGoal)),
                leds.errorCommand(),
                () -> isSafeToMove(intake));
        // () -> true);
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
            RobotStates.enableInitialized();
        }).until(bottomHallEffect::get).withName("Zero Encoder");
    }

    /**
     * Sets the speed of the elevator from the operator overrides controller.
     * 
     * @param speed  the speed commanded from the joystick
     * @param intake
     * @param leds
     * @return the command
     */
    public CommandBase setOverridenElevatorSpeedCommand(DoubleSupplier speed, Intake intake, LEDs leds) {
        return Commands.either(
                Commands.either(
                        run(() -> setSpeed(speed.getAsDouble())),
                        runOnce(this::stop).andThen(leds.errorCommand()),
                        () -> isSafeToMove(intake)),
                Commands.none(),
                Overrides::isOperatorOverridden).withName("Set Overridden Elevator Speed");
    }
}
