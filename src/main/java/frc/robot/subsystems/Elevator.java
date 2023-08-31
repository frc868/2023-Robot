package frc.robot.subsystems;

import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.techhounds.houndutil.houndlib.SparkMaxConfigurator;
import com.techhounds.houndutil.houndlib.Utils;
import com.techhounds.houndutil.houndlog.interfaces.Log;
import com.techhounds.houndutil.houndlog.interfaces.LoggedObject;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Overrides;
import frc.robot.Modes;
import frc.robot.GamePieceLocation.GamePiece;
import frc.robot.GamePieceLocation.Level;

/**
 * The elevator subsystem, with two motors and motion profling.
 * 
 * @author hr
 * @author jt
 */
@LoggedObject
public class Elevator extends SubsystemBase {
    public static enum ElevatorPosition {
        BOTTOM(0),
        CONE_LOW(0.25),
        CONE_MID(1.14),
        CONE_HIGH(1.7),
        CUBE_LOW(0.25),
        CUBE_MID(1.18482),
        SINGLE_SUBSTATION_PICKUP(0.5164),
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
    @Log(name = "Left Motor")
    private CANSparkMax leftMotor = new CANSparkMax(Constants.CAN.ELEVATOR_LEFT_MOTOR_ID, MotorType.kBrushless);

    /** The right motor of the elevator. */
    @Log(name = "Right Motor")
    private CANSparkMax rightMotor = new CANSparkMax(Constants.CAN.ELEVATOR_RIGHT_MOTOR_ID, MotorType.kBrushless);

    /** The group that controls both elevator motors. */
    private MotorControllerGroup motors = new MotorControllerGroup(leftMotor, rightMotor);

    /** Hall effect sensor that represents the minimum extension of the elevator. */
    @Log(name = "Bottom Hall Effect")
    private DigitalInput bottomHallEffect = new DigitalInput(Constants.DIO.ELEVATOR_BOTTOM_LIMIT);

    private TrapezoidProfile.Constraints normalConstraints = new TrapezoidProfile.Constraints(
            Constants.Geometries.Elevator.MAX_VELOCITY_METERS_PER_SECOND,
            Constants.Geometries.Elevator.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    private TrapezoidProfile.Constraints stowingConstraints = new TrapezoidProfile.Constraints(
            Constants.Geometries.Elevator.MAX_STOWING_VELOCITY_METERS_PER_SECOND,
            Constants.Geometries.Elevator.MAX_STOWING_ACCELERATION_METERS_PER_SECOND_SQUARED);

    @Log(name = "Profiled PID Controller")
    private ProfiledPIDController pidController = new ProfiledPIDController(
            Constants.Gains.Elevator.kP,
            Constants.Gains.Elevator.kI,
            Constants.Gains.Elevator.kD,
            normalConstraints);

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
     * The physics simulator for this mechanism. The elevator sim takes in voltages
     * from the motor and simulates an elevator under the influence of gravity.
     */
    private ElevatorSim elevatorSim = new ElevatorSim(
            DCMotor.getNEO(2),
            4,
            15,
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

    @Log(name = "Feedback", groups = "Control")
    private double feedback = 0;
    @Log(name = "Feedforward", groups = "Control")
    private double feedforward = 0;

    /**
     * Initializes the elevator.
     */
    public Elevator(MechanismLigament2d ligament) {
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

        pidController.setTolerance(Constants.Gains.Elevator.TOLERANCE);

        this.ligament = ligament;

        if (RobotBase.isSimulation()) {
            bottomHallEffectSim = new DIOSim(bottomHallEffect);
            bottomHallEffectSim.setValue(false);
        }

        new Trigger(bottomHallEffect::get).whileTrue(
                Commands.parallel(
                        Commands.runOnce(this::resetEncoders).ignoringDisable(true),
                        Commands.runOnce(Modes::enableInitialized)).ignoringDisable(true));

        setDefaultCommand(moveToCurrentGoalCommand());
    }

    /**
     * Runs every 20ms. This updates the ligament sent over NetworkTables.
     */
    @Override
    public void periodic() {
        ligament.setLength(-0.71 + getPosition());
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
    }

    /**
     * Returns the current position of the elevator.
     * 
     * @return the output of the left motor's encoder
     */
    @Log(name = "Position")
    public double getPosition() {
        return leftMotor.getEncoder().getPosition();
    }

    /**
     * Check if the elevator is safe to move.
     * 
     * @return true if the elevator is safe to move
     */
    private Pair<Boolean, String> getIfSafeToMove() {
        boolean safe = true;
        String str = "none";
        if (!Overrides.SAFETIES_DISABLE.getStatus()) {
            if (!Modes.isInitialized()) {
                safe = false;
                str = "Robot not initialized: cannot move elevator";
            }
        }

        return new Pair<Boolean, String>(safe, str);
    }

    /**
     * Sets the value of both motor encoders to zero.
     */
    public void resetEncoders() {
        leftMotor.getEncoder().setPosition(0);
        rightMotor.getEncoder().setPosition(0);
    }

    /**
     * Sets the speed of the motors. Will refuse to go past the bottom or top points
     * of the elevator.
     * 
     * @param speed the speed, from -1.0 to 1.0
     */
    public void setSpeed(double speed) {
        if (!Utils.limitMechanism(bottomHallEffect.get(), false, speed) || Overrides.MECH_LIMITS_DISABLE.getStatus()) {
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
        if (!Utils.limitMechanism(bottomHallEffect.get(), false,
                voltage) || Overrides.MECH_LIMITS_DISABLE.getStatus())
            motors.setVoltage(voltage);
        else {
            motors.setVoltage(0);
        }
    }

    private CommandBase moveToCurrentGoalCommand() {
        return run(() -> {
            feedback = pidController.calculate(getPosition());
            feedforward = feedforwardController.calculate(pidController.getSetpoint().position,
                    pidController.getSetpoint().velocity);
            setVoltage(feedback + feedforward);
        }).withName("Move to Current Goal");
    }

    public CommandBase moveToPositionCommand(Supplier<ElevatorPosition> goalPositionSupplier) {
        return Commands.either(
                Commands.sequence(
                        runOnce(() -> {
                            if (goalPositionSupplier.get() == ElevatorPosition.BOTTOM)
                                pidController.setConstraints(stowingConstraints);
                            else
                                pidController.setConstraints(normalConstraints);
                        }),
                        runOnce(() -> pidController.reset(getPosition())),
                        runOnce(() -> pidController.setGoal(goalPositionSupplier.get().value)), // also sets 0 velocity
                        moveToCurrentGoalCommand()
                                .until(pidController::atGoal))
                        .withTimeout(2)
                        .finallyDo((d) -> {
                            pidController.setConstraints(normalConstraints);
                            motors.stopMotor();
                        }),
                Modes.singularErrorCommand(() -> getIfSafeToMove().getSecond()),
                () -> getIfSafeToMove().getFirst());
    }

    public CommandBase moveToArbitraryPositionCommand(Supplier<Double> goalPositionSupplier) {
        return Commands.either(
                Commands.sequence(
                        runOnce(() -> pidController.reset(getPosition())),
                        runOnce(() -> pidController.setGoal(goalPositionSupplier.get())),
                        moveToCurrentGoalCommand().until(pidController::atGoal)).withTimeout(2)
                        .finallyDo((d) -> motors.stopMotor()),
                Modes.singularErrorCommand(() -> getIfSafeToMove().getSecond()),
                () -> getIfSafeToMove().getFirst());
    }

    public CommandBase movePositionDeltaCommand(Supplier<Double> delta) {
        return moveToArbitraryPositionCommand(() -> pidController.getGoal().position + delta.get());
    }

    public CommandBase holdCurrentPositionCommand() {
        return runOnce(() -> pidController.setGoal(getPosition())).andThen(run(() -> {
        }));
    }

    public CommandBase moveToScoringPositionCommand(Supplier<GamePiece> gamePieceMode,
            Supplier<Level> scoringMode, Intake intake, Elbow elbow) {
        return Commands.select(
                Map.of(
                        GamePiece.CONE,
                        Commands.select(
                                Map.of(
                                        Level.LOW,
                                        moveToPositionCommand(() -> ElevatorPosition.CONE_LOW),
                                        Level.MIDDLE,
                                        moveToPositionCommand(() -> ElevatorPosition.CONE_MID),
                                        Level.HIGH,
                                        moveToPositionCommand(() -> ElevatorPosition.CONE_HIGH)),
                                scoringMode::get),
                        GamePiece.CUBE,
                        Commands.select(
                                Map.of(
                                        Level.LOW,
                                        moveToPositionCommand(() -> ElevatorPosition.CUBE_LOW),
                                        Level.MIDDLE,
                                        moveToPositionCommand(() -> ElevatorPosition.CUBE_MID),
                                        Level.HIGH,
                                        moveToPositionCommand(() -> ElevatorPosition.CUBE_HIGH)),
                                scoringMode::get)),
                gamePieceMode::get);
    }

    /**
     * Creates a psuedo-StartEndCommand that runs the motors down at 15% speed until
     * the bottom hall effect sensor is reached, then zeros the encoders.
     */
    public CommandBase zeroEncoderCommand() {
        return startEnd(() -> setSpeed(-0.15), () -> {
            this.resetEncoders();
            Modes.enableInitialized();
            motors.stopMotor();
        }).until(bottomHallEffect::get).withName("Zero Encoder");
    }

    public CommandBase manualZeroEncoderCommand() {
        return Commands.runOnce(() -> {
            Modes.enableInitialized();
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
