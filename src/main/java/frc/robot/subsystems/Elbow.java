package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.techhounds.houndutil.houndlib.Utils;
import com.techhounds.houndutil.houndlog.LogGroup;
import com.techhounds.houndutil.houndlog.LogProfileBuilder;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.enums.LogLevel;
import com.techhounds.houndutil.houndlog.loggers.DeviceLogger;
import com.techhounds.houndutil.houndlog.logitems.BooleanLogItem;
import com.techhounds.houndutil.houndlog.logitems.DoubleLogItem;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Overrides;
import frc.robot.commands.RobotStates;

/**
 * Elbow subsystem, one CAN Motor and encoder, one feedforward object, and two
 * hall effect sensors.
 * 
 * @author jc
 */
public class Elbow extends ProfiledPIDSubsystem {
    public static enum ElbowPosition {
        LOW(0.502 - 0.94),
        MID(0.94 - 0.94),
        HIGH(1.7 - 0.94);

        public final double value;

        private ElbowPosition(double value) {
            this.value = value;
        }
    }

    /**
     * The motor that controls the elbow.
     */
    private CANSparkMax motor = new CANSparkMax(Constants.CAN.ELBOW_MOTOR, MotorType.kBrushless);

    /**
     * The encoder (connected to the Spark MAX) that detects the angle of the elbow.
     */
    private SparkMaxAbsoluteEncoder encoder = motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

    /**
     * The feedforward controller (calculates voltage based on the setpoint's
     * velocity and acceleration).
     */
    private ArmFeedforward feedforwardController = new ArmFeedforward(
            Constants.Gains.Elbow.kS,
            Constants.Gains.Elbow.kG,
            Constants.Gains.Elbow.kV,
            Constants.Gains.Elbow.kA);

    /** Hall effect sensor that represents the lowest angle of the elbow. */
    private DigitalInput bottomHallEffect = new DigitalInput(Constants.DIO.ELBOW_BOTTOM_LIMIT);
    /** Hall effect sensor that represents the highest angle of the elbow. */
    private DigitalInput topHallEffect = new DigitalInput(Constants.DIO.ELBOW_TOP_LIMIT);

    /** The ligament of the complete mechanism body that this subsystem controls. */
    private MechanismLigament2d ligament;

    /**
     * The physics simulator for this mechansim. The single jointed arm sim best
     * matches the elbow.
     */
    private SingleJointedArmSim armSim = new SingleJointedArmSim(
            DCMotor.getNEO(2),
            100,
            SingleJointedArmSim.estimateMOI(Units.inchesToMeters(8), 10),
            Units.inchesToMeters(8),
            ElbowPosition.LOW.value - 0.25,
            ElbowPosition.HIGH.value + 0.25,
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
     * Initializes the elbow.
     */
    public Elbow(MechanismLigament2d ligament) {
        super(new ProfiledPIDController(
                Constants.Gains.Elbow.kP.get(),
                Constants.Gains.Elbow.kI.get(),
                Constants.Gains.Elbow.kD.get(),
                new TrapezoidProfile.Constraints(
                        Constants.Geometries.Elbow.MAX_VELOCITY_METERS_PER_SECOND.get(),
                        Constants.Geometries.Elbow.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED.get())));

        getController().setTolerance(Constants.Gains.Elbow.TOLERANCE.get());
        getController().enableContinuousInput(0, 2 * Math.PI);

        Constants.Gains.Elbow.kP.setConsumer((d) -> getController().setP(d));
        Constants.Gains.Elbow.kI.setConsumer((d) -> getController().setI(d));
        Constants.Gains.Elbow.kD.setConsumer((d) -> getController().setD(d));
        Constants.Gains.Elbow.TOLERANCE.setConsumer((d) -> getController().setTolerance(d));
        Constants.Geometries.Elbow.MAX_VELOCITY_METERS_PER_SECOND.setConsumer((d) -> getController()
                .setConstraints(
                        new TrapezoidProfile.Constraints(
                                d,
                                Constants.Geometries.Elbow.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED.get())));

        Constants.Geometries.Elbow.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED.setConsumer((d) -> getController()
                .setConstraints(
                        new TrapezoidProfile.Constraints(
                                Constants.Geometries.Elbow.MAX_VELOCITY_METERS_PER_SECOND.get(),
                                d)));

        this.ligament = ligament;

        // SparkMaxConfigurator.configure(motor, false)
        // .withIdleMode(IdleMode.kBrake)
        // .withCurrentLimit(20)
        // .withInverted(true)
        // .withPositionConversionFactor(2 * Math.PI / 100.0, true)
        // .burnFlash();

        motor.setInverted(true);
        motor.getEncoder().setPositionConversionFactor(2 * Math.PI / 100.0);
        motor.getEncoder().setVelocityConversionFactor(2 * Math.PI / 100.0 / 60.0);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);

        encoder.setInverted(true);
        encoder.setPositionConversionFactor(2 * Math.PI);
        encoder.setZeroOffset(1.26);

        motor.burnFlash();

        LoggingManager.getInstance().addGroup("Elbow", new LogGroup(
                new BooleanLogItem("Bottom Hall Effect", bottomHallEffect::get, LogLevel.MAIN),
                new BooleanLogItem("Top Hall Effect", topHallEffect::get, LogLevel.MAIN),
                new DoubleLogItem("Actual Position", () -> encoder.getPosition(), LogLevel.MAIN),
                new DoubleLogItem("Actual Velocity", () -> encoder.getVelocity(), LogLevel.MAIN),
                new DoubleLogItem("Setpoint Position", () -> setpointPosition, LogLevel.MAIN),
                new DoubleLogItem("Setpoint Velocity", () -> setpointVelocity, LogLevel.MAIN),
                new DoubleLogItem("Feedforward", () -> feedforward, LogLevel.MAIN),
                new DoubleLogItem("PID Output", () -> pidOutput, LogLevel.MAIN),
                new DeviceLogger<CANSparkMax>(motor, "Elbow Motor",
                        LogProfileBuilder.buildCANSparkMaxLogItems(motor))));

        new Trigger(bottomHallEffect::get)
                .whileTrue(RobotStates.continuousErrorCommand(() -> "Bottom elbow limit triggered"));
        new Trigger(topHallEffect::get)
                .whileTrue(RobotStates.continuousErrorCommand(() -> "Top elbow limit triggered"));

        if (RobotBase.isSimulation()) {
            bottomHallEffectSim = new DIOSim(bottomHallEffect);
            bottomHallEffectSim.setValue(false);
            topHallEffectSim = new DIOSim(topHallEffect);
            topHallEffectSim.setValue(false);
        }
    }

    /**
     * Runs every 20ms. This updates the ligament sent over NetworkTables.
     */
    @Override
    public void periodic() {
        super.periodic();
        ligament.setAngle(-34 + Units.radiansToDegrees(motor.getEncoder().getPosition()));
    }

    /**
     * Updates simulation-specific variables.
     */
    @Override
    public void simulationPeriodic() {
        armSim.setInput(motor.getAppliedOutput());
        armSim.update(0.020);
        motor.getEncoder().setPosition(armSim.getAngleRads());
        // update the voltage of the RIO sim
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));

        bottomHallEffectSim.setValue(armSim.getAngleRads() <= ElbowPosition.LOW.value - 0.25);
        topHallEffectSim.setValue(armSim.getAngleRads() >= ElbowPosition.HIGH.value + 0.25);
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
    protected void useOutput(double output, State setpoint) {
        setpointPosition = setpoint.position;
        setpointVelocity = setpoint.velocity;
        feedforward = feedforwardController.calculate(setpoint.position, setpoint.velocity);
        pidOutput = output;
        setVoltage(output + feedforwardController.calculate(setpoint.position, setpoint.velocity));
    }

    /**
     * Returns the measurement of the process variable used by the
     * ProfiledPIDController.
     * 
     * @return the encoder output
     */
    @Override
    protected double getMeasurement() {
        return RobotBase.isReal() ? encoder.getPosition() : motor.getEncoder().getPosition();
    }

    /**
     * Get the goal position of the elbow.
     * 
     * @return the goal position of the elbow
     */
    public double getGoal() {
        return getController().getGoal().position;
    }

    /**
     * Checks if the elbow is at its goal position.
     * 
     * @return true if the elbow is at its goal position
     */
    public boolean isAtGoal() {
        return getController().atGoal();
    }

    /**
     * Check if the elbow is safe to move to a specific position.
     * 
     * @return true if the elbow is safe to move
     */
    private Pair<Boolean, String> getIfSafeToMove(ElbowPosition targetPosition, Elevator elevator) {
        boolean safe = true;
        String str = "none";

        if (!Overrides.SAFETIES_DISABLE.getStatus()) {
            if (!RobotStates.isInitialized()) {
                safe = false;
                str = "Robot not initialized: cannot move elbow";
            }

            switch (targetPosition) {
                case HIGH:
                    if (elevator.getMeasurement() < 0.2) {
                        safe = false;
                        str = "Elbow not clear of intake: cannot move elbow to high position";
                    }
                    break;
                case MID:
                    break;
                case LOW:
                    if (elevator.getMeasurement() < 0.1) {
                        safe = false;
                        str = "Elevator too low: cannot move elbow to low position";
                    }
            }
        }
        return new Pair<Boolean, String>(safe, str);
    }

    /**
     * Sets the speed of the motor. Will refuse to go past the bottom or top points
     * of the elbow.
     * 
     * @param speed the speed, from -1.0 to 1.0
     */
    private void setSpeed(double speed) {
        if (Math.abs(speed) > 8) {
            speed = Math.copySign(8, speed);
        }
        if (!Utils.limitMechanism(bottomHallEffect.get(), false,
                speed) || Overrides.MECH_LIMITS_DISABLE.getStatus()) {
            if (RobotBase.isReal())
                motor.set(speed);
            else
                motor.setVoltage(speed * 12.0);
        } else {
            motor.setVoltage(0);
        }
    }

    /**
     * Sets the voltage of the motor. Will refuse to go past the bottom or top
     * points of the elbow.
     * 
     * @param voltage the voltage, from -12.0v to 12.0v
     */
    private void setVoltage(double voltage) {
        if (!Utils.limitMechanism(bottomHallEffect.get(), false,
                voltage) || Overrides.MECH_LIMITS_DISABLE.getStatus()) {

            if (Math.abs(voltage) > 8) {
                voltage = Math.copySign(8, voltage);
            }
            motor.setVoltage(voltage);
        } else {
            motor.setVoltage(0);
        }
    }

    /**
     * Returns an InstantCommand that sets the goal position of the elbow, and
     * enables the controller.
     * 
     * @param position an ElbowPosition to set the elbow to
     * @return the command
     */
    public CommandBase setDesiredPositionCommand(ElbowPosition position, Elevator elevator) {
        return Commands.either(
                Commands.sequence(
                        runOnce(() -> setGoal(position.value)),
                        runOnce(this::enable),
                        Commands.waitUntil(this::isAtGoal).withTimeout(0.5)),
                RobotStates.singularErrorCommand(() -> getIfSafeToMove(position, elevator).getSecond()),
                () -> getIfSafeToMove(position, elevator).getFirst());
    }

    public CommandBase lockPosition() {
        // set the voltage to the feedforward calculated kg, then idle, then reenable
        // the controller
        return Commands.runOnce(this::disable)
                .andThen(Commands.runOnce(() -> setVoltage(feedforwardController.kg * Math.cos(getMeasurement()))))
                .andThen(run(() -> { // idle
                })).finallyDo(d -> this.enable());
    }

    /**
     * Sets the speed of the elbow from the operator overrides controller.
     * 
     * @param speed the speed commanded from the joystick
     * @return the command
     */
    public CommandBase setOverridenElbowSpeedCommand(DoubleSupplier speed) {
        return Commands.either(
                run(() -> setSpeed(speed.getAsDouble())),
                Commands.none(),
                Overrides.MANUAL_MECH_CONTROL_MODE.getStatusSupplier());
    }

    public CommandBase motorOverride() {
        return new FunctionalCommand(
                () -> {
                    motor.setIdleMode(IdleMode.kCoast);
                },
                () -> {
                    motor.stopMotor();
                },
                (d) -> {
                    motor.setIdleMode(IdleMode.kBrake);
                },
                () -> false,
                this).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }
}
