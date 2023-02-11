package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.techhounds.houndutil.houndlib.Utils;
import com.techhounds.houndutil.houndlog.LogGroup;
import com.techhounds.houndutil.houndlog.LogProfileBuilder;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.enums.LogLevel;
import com.techhounds.houndutil.houndlog.loggers.DeviceLogger;
import com.techhounds.houndutil.houndlog.logitems.BooleanLogItem;
import com.techhounds.houndutil.houndlog.logitems.DoubleLogItem;

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
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;
import frc.robot.Overrides;

/**
 * Elbow subsystem, one CAN Motor and encoder, one feedforward object, and two
 * hall effect sensors.
 * 
 * @author jc
 */
public class Elbow extends ProfiledPIDSubsystem {
    public static enum ElbowPosition {
        LOW(Units.degreesToRadians(-20)),
        MID(0),
        HIGH(Units.degreesToRadians(20));

        public final double value;

        private ElbowPosition(double value) {
            this.value = value;
        }
    }

    /**
     * The motor that controls the elbow.
     */
    private CANSparkMax motor = new CANSparkMax(Constants.Elbow.CANIDs.MOTOR, MotorType.kBrushless);

    /**
     * The encoder (connected to the Spark MAX) that detects the angle of the elbow.
     */
    private SparkMaxAbsoluteEncoder encoder = motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

    /**
     * The feedforward controller (calculates voltage based on the setpoint's
     * velocity and acceleration).
     */
    private ArmFeedforward feedforwardController = new ArmFeedforward(
            Constants.Elbow.Gains.kS,
            Constants.Elbow.Gains.kG,
            Constants.Elbow.Gains.kV,
            Constants.Elbow.Gains.kA);

    /** Hall effect sensor that represents the lowest angle of the elbow. */
    private DigitalInput bottomHallEffect = new DigitalInput(Constants.Elbow.BOTTOM_HALL_EFFECT_PORT);
    /** Hall effect sensor that represents the highest angle of the elbow. */
    private DigitalInput topHallEffect = new DigitalInput(Constants.Elbow.TOP_HALL_EFFECT_PORT);

    /** The ligament of the complete mechanism body that this subsystem controls. */
    private MechanismLigament2d ligament;

    /**
     * The physics simulator for this mechansim. The single jointed arm sim best
     * matches the elbow.
     */
    private SingleJointedArmSim armSim = new SingleJointedArmSim(
            DCMotor.getNEO(2),
            100,
            SingleJointedArmSim.estimateMOI(Units.inchesToMeters(8), 3.6),
            Units.inchesToMeters(8),
            Units.degreesToRadians(-30),
            Units.degreesToRadians(30),
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
     * Initializes the elbow.
     */
    public Elbow(MechanismLigament2d ligament) {
        super(new ProfiledPIDController(
                Constants.Elbow.Gains.kP.get(),
                Constants.Elbow.Gains.kP.get(),
                Constants.Elbow.Gains.kP.get(),
                new TrapezoidProfile.Constraints(
                        Constants.Elbow.MAX_VELOCITY_METERS_PER_SECOND.get(),
                        Constants.Elbow.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED.get())));

        getController().setTolerance(Constants.Elbow.Gains.TOLERANCE.get());

        Constants.Elbow.Gains.kP.setConsumer((d) -> getController().setP(d));
        Constants.Elbow.Gains.kI.setConsumer((d) -> getController().setI(d));
        Constants.Elbow.Gains.kD.setConsumer((d) -> getController().setD(d));
        Constants.Elbow.Gains.TOLERANCE.setConsumer((d) -> getController().setTolerance(d));
        Constants.Elbow.MAX_VELOCITY_METERS_PER_SECOND.setConsumer((d) -> getController()
                .setConstraints(
                        new TrapezoidProfile.Constraints(
                                d,
                                Constants.Elbow.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED.get())));

        Constants.Elbow.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED.setConsumer((d) -> getController()
                .setConstraints(
                        new TrapezoidProfile.Constraints(
                                Constants.Elbow.MAX_VELOCITY_METERS_PER_SECOND.get(),
                                d)));

        this.ligament = ligament;

        LoggingManager.getInstance().addGroup("Elbow", new LogGroup(
                new BooleanLogItem("Bottom Hall Effect", bottomHallEffect::get),
                new BooleanLogItem("Top Hall Effect", topHallEffect::get),
                new DoubleLogItem("Actual Position", () -> this.getMeasurement(), LogLevel.MAIN),
                new DeviceLogger<CANSparkMax>(motor, "Elbow Motor",
                        LogProfileBuilder.buildCANSparkMaxLogItems(motor))));

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
        if (RobotBase.isReal())
            ligament.setAngle(-42 + Units.radiansToDegrees(encoder.getPosition()));
        else
            ligament.setAngle(-42 + Units.radiansToDegrees(motor.getEncoder().getPosition()));
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

        bottomHallEffectSim.setValue(armSim.getAngleRads() <= Units.degreesToRadians(-30));
        topHallEffectSim.setValue(armSim.getAngleRads() >= Units.degreesToRadians(30));
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
        motor.setVoltage(output + feedforwardController.calculate(setpoint.position, setpoint.velocity));
    }

    /**
     * Returns the measurement of the process variable used by the
     * ProfiledPIDController.
     * 
     * @return the encoder output
     */
    @Override
    protected double getMeasurement() {
        if (RobotBase.isReal())
            return encoder.getPosition();
        else
            return motor.getEncoder().getPosition();
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
     * Sets the speed of the motor. Will refuse to go past the bottom or top points
     * of the elbow.
     * 
     * @param speed the speed, from -1.0 to 1.0
     */
    public void setSpeed(double speed) {
        if (!Utils.limitMechanism(bottomHallEffect.get(), topHallEffect.get(), speed))
            if (RobotBase.isReal())
                motor.set(speed);
            else
                motor.setVoltage(speed * 12.0);
        else {
            motor.setVoltage(0);
        }
    }

    /**
     * Sets the voltage of the motor. Will refuse to go past the bottom or top
     * points of the elbow.
     * 
     * @param voltage the voltage, from -12.0v to 12.0v
     */
    public void setVoltage(double voltage) {
        if (!Utils.limitMechanism(bottomHallEffect.get(), topHallEffect.get(), voltage))
            motor.setVoltage(voltage);
        else {
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
    public CommandBase setDesiredPositionCommand(ElbowPosition position) {
        return Commands.sequence(
                runOnce(() -> setGoal(position.value)),
                runOnce(this::enable),
                Commands.waitUntil(this::isAtGoal));
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
                Overrides::isOperatorOverridden);
    }
}
