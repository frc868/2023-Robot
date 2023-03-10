package frc.robot.subsystems;

import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.techhounds.houndutil.houndlib.SparkMaxConfigurator;
import com.techhounds.houndutil.houndlib.Utils;
import com.techhounds.houndutil.houndlog.LogGroup;
import com.techhounds.houndutil.houndlog.LogProfileBuilder;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.enums.LogLevel;
import com.techhounds.houndutil.houndlog.loggers.DeviceLogger;
import com.techhounds.houndutil.houndlog.loggers.Logger;
import com.techhounds.houndutil.houndlog.logitems.BooleanLogItem;
import com.techhounds.houndutil.houndlog.logitems.DoubleLogItem;

import edu.wpi.first.math.Pair;
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
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
        CONE_LOW(0.25),
        CONE_MID(1.14),
        CONE_HIGH(1.7),
        CUBE_LOW(0.25),
        CUBE_MID(1.18482),
        DOUBLE_SUBSTATION_PICKUP(1.664),
        CUBE_HIGH(1.70220),
        HUMAN_PLAYER(0),
        TOP(1.40);
        // 1.72 meters max

        public final double value;

        private ElevatorPosition(double value) {
            this.value = value;
        }
    }

    /** The left motor of the elevator. */
    private CANSparkMax leftMotor = new CANSparkMax(Constants.CAN.ELEVATOR_LEFT_MOTOR, MotorType.kBrushless);

    /** The right motor of the elevator. */
    private CANSparkMax rightMotor = new CANSparkMax(Constants.CAN.ELEVATOR_RIGHT_MOTOR, MotorType.kBrushless);

    /** The object that controlls both elevator motors. */
    private MotorControllerGroup motors = new MotorControllerGroup(leftMotor, rightMotor);

    /** Hall effect sensor that represents the minimum extension of the elevator. */
    private DigitalInput bottomHallEffect = new DigitalInput(Constants.DIO.ELEVATOR_BOTTOM_LIMIT);

    /** Hall effect sensor that represents the maximum extension of the elevator. */
    private DigitalInput topHallEffect = new DigitalInput(Constants.DIO.ELEVATOR_TOP_LIMIT);

    /**
     * The feedforward controller (calculates voltage based on the setpoint's
     * velocity and acceleration).
     */
    private ElevatorFeedforward feedforwardController = new ElevatorFeedforward(
            Constants.Gains.Elevator.kS,
            Constants.Gains.Elevator.kG,
            Constants.Gains.Elevator.kV,
            Constants.Gains.Elevator.kA);

    /** The ligament of the complete mechanism body that this subsystem controls. */
    private MechanismLigament2d ligament;

    /**
     * The physics simulator for this mechansim. The elevator sim takes in voltages
     * from the motor and simulates an elevator under the influence of gravity.
     */
    private ElevatorSim elevatorSim = new ElevatorSim(
            DCMotor.getNEO(2),
            4,
            10,
            Units.inchesToMeters(1.2),
            Units.inchesToMeters(0),
            1.72,
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

    /** Used to graph on NT. */
    private double setpointPosition = 0;
    /** Used to graph on NT. */
    private double setpointVelocity = 0;
    /** Used to graph on NT. */
    private double pidOutput = 0;
    /** Used to graph on NT. */
    private double feedforward = 0;

    /**
     * Initializes the elevator.
     */
    public Elevator(MechanismLigament2d ligament) {
        super(new ProfiledPIDController(
                Constants.Gains.Elevator.kP.get(),
                Constants.Gains.Elevator.kI.get(),
                Constants.Gains.Elevator.kD.get(),
                // 0, 0, 0,
                new TrapezoidProfile.Constraints(
                        Constants.Geometries.Elevator.MAX_VELOCITY_METERS_PER_SECOND.get(),
                        Constants.Geometries.Elevator.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED.get())));

        getController().setTolerance(Constants.Gains.Elevator.TOLERANCE.get());

        Constants.Gains.Elevator.kP.setConsumer((d) -> getController().setP(d));
        Constants.Gains.Elevator.kI.setConsumer((d) -> getController().setI(d));
        Constants.Gains.Elevator.kD.setConsumer((d) -> getController().setD(d));
        Constants.Gains.Elevator.TOLERANCE.setConsumer((d) -> getController().setTolerance(d));
        // Constants.Geometries.Elevator.MAX_VELOCITY_METERS_PER_SECOND.setConsumer((d)
        // -> getController()
        // .setConstraints(
        // new TrapezoidProfile.Constraints(
        // d,
        // Constants.Geometries.Elevator.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED.get())));

        // Constants.Geometries.Elevator.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED.setConsumer((d)
        // -> getController()
        // .setConstraints(
        // new TrapezoidProfile.Constraints(
        // Constants.Geometries.Elevator.MAX_VELOCITY_METERS_PER_SECOND.get(),
        // d)));

        this.ligament = ligament;

        SparkMaxConfigurator.configure(leftMotor)
                .withIdleMode(IdleMode.kBrake)
                .withCurrentLimit(40)
                .withPositionConversionFactor(Constants.Geometries.Elevator.ENCODER_DISTANCE_TO_METERS, true)
                .burnFlash();

        SparkMaxConfigurator.configure(rightMotor)
                .withIdleMode(IdleMode.kBrake)
                .withCurrentLimit(40)
                .withInverted(true)
                .withPositionConversionFactor(Constants.Geometries.Elevator.ENCODER_DISTANCE_TO_METERS, true)
                .burnFlash();

        LoggingManager.getInstance().addGroup("Elevator", new LogGroup(
                new Logger[] {
                        new BooleanLogItem("Bottom Hall Effect", bottomHallEffect::get, LogLevel.MAIN),
                        new BooleanLogItem("Top Hall Effect", topHallEffect::get, LogLevel.MAIN),
                        new DoubleLogItem("Control/Position", () -> this.getMeasurement(), LogLevel.MAIN),
                        new DoubleLogItem("Control/Velocity", () -> leftMotor.getEncoder().getVelocity(),
                                LogLevel.MAIN),
                        new DoubleLogItem("Control/Setpoint Position", () -> setpointPosition, LogLevel.MAIN),
                        new DoubleLogItem("Control/Setpoint Velocity", () -> setpointVelocity, LogLevel.MAIN),
                        new DoubleLogItem("Control/Feedforward", () -> feedforward, LogLevel.MAIN),
                        new DoubleLogItem("Control/PID Output", () -> pidOutput, LogLevel.MAIN),
                        new DoubleLogItem("Control/Total Output", () -> pidOutput + feedforward, LogLevel.MAIN),
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
            // RobotStates.enableInitialized();
        }

        new Trigger(bottomHallEffect::get).whileTrue(
                Commands.parallel(
                        Commands.runOnce(this::resetEncoders).ignoringDisable(true),
                        Commands.runOnce(RobotStates::enableInitialized)).ignoringDisable(true));
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
        setpointPosition = setpoint.position;
        setpointVelocity = setpoint.velocity;
        feedforward = feedforwardController.calculate(setpoint.velocity);
        pidOutput = output;
        setVoltage(output + feedforward);
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

    public boolean isAtPosition(ElevatorPosition position) {
        return isEnabled()
                ? this.getGoal() == position.value && isAtGoal()
                : (this.getMeasurement() - position.value) < Constants.Gains.Elevator.TOLERANCE.get();
    }

    /**
     * Check if the elevator is lowered. This is useful for the subsystem safeties.
     * 
     * @return true if the elevator is at the bottom state and is within the
     *         tolerance of the controller.
     */
    public Pair<Boolean, String> isSafeForIntake() {
        boolean safe = isAtPosition(ElevatorPosition.BOTTOM);
        String str = safe ? "none" : "Elevator not lowered: cannot move intake";
        return new Pair<Boolean, String>(safe, str);
    }

    /**
     * Check if the elevator is safe to move.
     * 
     * @return true if the elevator is safe to move
     */
    private Pair<Boolean, String> getIfSafeToMove(Intake intake, Elbow elbow) {
        boolean safe = true;
        String str = "none";
        if (!Overrides.SAFETIES_DISABLE.getStatus()) {

            if (!RobotStates.isInitialized()) {
                safe = false;
                str = "Robot not initialized: cannot move elevator";
            }

            if (!intake.isSafeForElevator().getFirst()) {
                safe = false;
                str = intake.isSafeForElevator().getSecond();
            }
        }

        return new Pair<Boolean, String>(safe, str);
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
        if (!Utils.limitMechanism(bottomHallEffect.get(), topHallEffect.get(),
                speed) || Overrides.MECH_LIMITS_DISABLE.getStatus()) {
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
    private void setVoltage(double voltage) {
        if (!Utils.limitMechanism(bottomHallEffect.get(), topHallEffect.get(),
                voltage) || Overrides.MECH_LIMITS_DISABLE.getStatus())
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
    public CommandBase setDesiredPositionCommand(ElevatorPosition position, Intake intake,
            Elbow elbow) {
        return setDesiredPositionCommand(() -> position, intake, elbow);
    }

    public CommandBase setDesiredPositionCommand(Supplier<ElevatorPosition> positionSupplier, Intake intake,
            Elbow elbow) {
        return Commands.either(
                Commands.sequence(
                        runOnce(() -> {
                            if (positionSupplier.get() == ElevatorPosition.BOTTOM)
                                getController().setConstraints(new TrapezoidProfile.Constraints(
                                        Constants.Geometries.Elevator.MAX_VELOCITY_METERS_PER_SECOND_STOW.get(),
                                        Constants.Geometries.Elevator.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED_STOW
                                                .get()));
                            else
                                getController().setConstraints(new TrapezoidProfile.Constraints(
                                        Constants.Geometries.Elevator.MAX_VELOCITY_METERS_PER_SECOND.get(),
                                        Constants.Geometries.Elevator.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED
                                                .get()));
                        }),
                        runOnce(() -> setGoal(positionSupplier.get().value)),
                        runOnce(this::enable),
                        Commands.waitUntil(this::isAtGoal).withTimeout(1.5)),
                RobotStates.singularErrorCommand(() -> getIfSafeToMove(intake, elbow).getSecond()),
                () -> getIfSafeToMove(intake, elbow).getFirst());
    }

    /**
     * Creates an InstantCommand that drops the current position of the elevator by
     * a value.
     * 
     * @param position an ElevatorPosition to set the elevator to
     * @return the command
     */
    public CommandBase setDesiredPositionDeltaCommand(double position, Intake intake, Elbow elbow) {
        return Commands.either(
                Commands.sequence(
                        runOnce(() -> {
                            getController().setConstraints(new TrapezoidProfile.Constraints(
                                    Constants.Geometries.Elevator.MAX_VELOCITY_METERS_PER_SECOND_STOW.get(),
                                    Constants.Geometries.Elevator.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED_STOW
                                            .get()));
                        }),
                        runOnce(() -> setGoal(getGoal() + position)),
                        runOnce(this::enable),
                        Commands.waitUntil(this::isAtGoal).withTimeout(0.3)),
                RobotStates.singularErrorCommand(() -> getIfSafeToMove(intake, elbow).getSecond()),
                () -> getIfSafeToMove(intake, elbow).getFirst());
    }

    public CommandBase setScoringPositionCommand(Supplier<GamePiece> gamePieceMode,
            Supplier<Level> scoringMode, Intake intake, Elbow elbow) {
        return Commands.select(
                Map.of(
                        GamePiece.CONE,
                        Commands.select(
                                Map.of(
                                        Level.LOW,
                                        setDesiredPositionCommand(ElevatorPosition.CONE_LOW, intake, elbow),
                                        Level.MIDDLE,
                                        setDesiredPositionCommand(ElevatorPosition.CONE_MID, intake, elbow),
                                        Level.HIGH,
                                        setDesiredPositionCommand(ElevatorPosition.CONE_HIGH, intake, elbow)),
                                scoringMode::get),
                        GamePiece.CUBE,
                        Commands.select(
                                Map.of(
                                        Level.LOW,
                                        setDesiredPositionCommand(ElevatorPosition.CUBE_LOW, intake, elbow),
                                        Level.MIDDLE,
                                        setDesiredPositionCommand(ElevatorPosition.CUBE_MID, intake, elbow),
                                        Level.HIGH,
                                        setDesiredPositionCommand(ElevatorPosition.CUBE_HIGH, intake, elbow)),
                                scoringMode::get)),
                gamePieceMode::get);
    }

    /**
     * Creates a psuedo-StartEndCommand that runs the motors down at 10% speed until
     * the bottom hall effect sensor is reached, then zeros the encoders.
     */
    public CommandBase zeroEncoderCommand() {
        return startEnd(() -> setSpeed(-0.2), () -> {
            this.resetEncoders();
            RobotStates.enableInitialized();
            motors.stopMotor();
        }).until(bottomHallEffect::get).withName("Zero Encoder");
    }

    public CommandBase manualZeroEncoderCommand() {
        return Commands.runOnce(() -> {
            RobotStates.enableInitialized();
            this.resetEncoders();
        });
    }

    /**
     * Sets the speed of the elevator from the operator overrides controller.
     * 
     * @param speed  the speed commanded from the joystick
     * @param intake
     * @param leds
     * @return the command
     */
    public CommandBase setOverridenElevatorSpeedCommand(DoubleSupplier speed, Intake intake, Elbow elbow) {
        return run(() -> {
            if (Overrides.MANUAL_MECH_CONTROL_MODE.getStatus()) {
                this.disable();
                setSpeed(speed.getAsDouble());
            }
        }).withName("Set Overridden Elevator Speed");
    }

    public CommandBase motorOverride() {
        return new FunctionalCommand(
                () -> {
                    leftMotor.setIdleMode(IdleMode.kCoast);
                    rightMotor.setIdleMode(IdleMode.kCoast);
                },
                () -> {
                    motors.stopMotor();
                },
                (d) -> {
                    leftMotor.setIdleMode(IdleMode.kBrake);
                    rightMotor.setIdleMode(IdleMode.kBrake);
                },
                () -> false,
                this).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }
}
