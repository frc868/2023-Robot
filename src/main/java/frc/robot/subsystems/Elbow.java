package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.techhounds.houndutil.houndlib.SparkMaxConfigurator;
import com.techhounds.houndutil.houndlog.interfaces.Log;
import com.techhounds.houndutil.houndlog.interfaces.LoggedObject;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.Constants;
import frc.robot.Overrides;

/**
 * The elbow subsystem, controlling the elbow's position.
 * 
 * @author jc
 * @author dr
 */
@LoggedObject
public class Elbow extends SubsystemBase {
    public static enum ElbowPosition {
        LOW(-0.31),
        MID_STOW(-0.1),
        MID(-0.05),
        MID_CONE_HIGH(0.3),
        CONE_PICKUP(0.07),
        SINGLE_SUBSTATION_PICKUP(0.087),
        HIGH(0.75);

        public final double value;

        private ElbowPosition(double value) {
            this.value = value;
        }
    }

    @Log(name = "Motor")
    private CANSparkMax motor = new CANSparkMax(Constants.CAN.ELBOW_MOTOR_ID, MotorType.kBrushless);
    @Log(name = "Encoder")
    private SparkMaxAbsoluteEncoder encoder = motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

    private MechanismLigament2d ligament;

    private SingleJointedArmSim armSim = new SingleJointedArmSim(
            DCMotor.getNeo550(1),
            100,
            SingleJointedArmSim.estimateMOI(Units.inchesToMeters(12), 3.63),
            Units.inchesToMeters(12),
            ElbowPosition.LOW.value - 0.25,
            ElbowPosition.HIGH.value + 0.25,
            true);

    @Log(name = "Profiled PID Controller")
    private ProfiledPIDController pidController = new ProfiledPIDController(
            Constants.Gains.Elbow.kP,
            Constants.Gains.Elbow.kI,
            Constants.Gains.Elbow.kD,
            new TrapezoidProfile.Constraints(
                    Constants.Geometries.Elbow.MAX_VELOCITY_METERS_PER_SECOND,
                    Constants.Geometries.Elbow.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED));

    @Log(name = "Feedforward Controller")
    private ArmFeedforward feedforwardController = new ArmFeedforward(
            Constants.Gains.Elbow.kS,
            Constants.Gains.Elbow.kG,
            Constants.Gains.Elbow.kV,
            Constants.Gains.Elbow.kA);

    public Elbow(MechanismLigament2d ligament) {
        this.ligament = ligament;

        SparkMaxConfigurator.configure(motor, false)
                .withIdleMode(IdleMode.kBrake)
                .withCurrentLimit(20)
                .withInverted(true)
                .withPositionConversionFactor(2 * Math.PI / 100.0, true);

        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
        encoder.setInverted(true);
        encoder.setPositionConversionFactor(2 * Math.PI);
        encoder.setZeroOffset(1.274);
        motor.burnFlash();

        new Thread(() -> {
            try {
                Thread.sleep(3000);
                motor.getEncoder().setPosition(encoder.getPosition());
            } catch (Exception e) {
            }
        }).start();

        setDefaultCommand(moveToCurrentGoalCommand());
    }

    @Override
    public void periodic() {
        ligament.setAngle(-34 + Units.radiansToDegrees(motor.getEncoder().getPosition()));
    }

    @Override
    public void simulationPeriodic() {
        armSim.setInput(motor.getAppliedOutput());
        armSim.update(0.020);
        motor.getEncoder().setPosition(armSim.getAngleRads());
        // update the voltage of the RIO sim
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));
    }

    @Log(name = "Position")
    private double getPosition() {
        return motor.getEncoder().getPosition();
    }

    /**
     * Sets the speed of the motor. Will refuse to go past the bottom or top soft
     * limits.
     * 
     * @param speed the speed, from -1.0 to 1.0.
     */
    private void setSpeed(double speed) {
        if (RobotBase.isReal())
            motor.set(speed);
        else
            motor.setVoltage(speed * 12.0);
    }

    /**
     * Sets the voltage of the motor.
     * 
     * @param voltage the voltage, from -12.0v to 12.0v
     */
    private void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    /**
     * Creates a command that moves to the currently specified goal pose, then holds
     * its position.
     * 
     * @return the command
     */
    private CommandBase moveToCurrentGoalCommand() {
        return run(() -> {
            double feedback = pidController.calculate(getPosition());
            double feedforward = feedforwardController.calculate(pidController.getSetpoint().position,
                    pidController.getSetpoint().velocity);
            setVoltage(feedback + feedforward);
        }).withName("Move to Current Goal");
    }

    /**
     * Creates a command that sets a new goal position, and waits until the elbow
     * has
     * reached its goal.
     * 
     * @param goalPositionSupplier the supplier for the new goal position
     * @return the command
     */
    public CommandBase moveToPositionCommand(Supplier<ElbowPosition> goalPositionSupplier) {
        return Commands.sequence(
                runOnce(() -> pidController.reset(getPosition())),
                runOnce(() -> pidController.setGoal(goalPositionSupplier.get().value)),
                moveToCurrentGoalCommand().until(pidController::atGoal)).withTimeout(2)
                .finallyDo((d) -> motor.stopMotor())
                .withName("Move to Position");
    }

    /**
     * Creates a command that sets a new arbitrary goal position, and waits until
     * the elbow has
     * reached its goal.
     * 
     * @param goalPositionSupplier the supplier for the new goal position
     * @return the command
     */
    public CommandBase moveToArbitraryPositionCommand(Supplier<Double> goalPositionSupplier) {
        return Commands.sequence(
                runOnce(() -> pidController.reset(getPosition())),
                runOnce(() -> pidController.setGoal(goalPositionSupplier.get())),
                moveToCurrentGoalCommand().until(pidController::atGoal)).withTimeout(2)
                .finallyDo((d) -> motor.stopMotor())
                .withName("Move to Arbitrary Position");
    }

    public CommandBase holdCurrentPositionCommand() {
        return runOnce(() -> pidController.setGoal(getPosition())).andThen(run(() -> {
        }));
    }

    /**
     * Sets the speed of the elbow from the operator overrides controller.
     * 
     * @param speed the speed commanded from the joystick
     * @return the command
     */
    // TODO
    public CommandBase setOverridenElbowSpeedCommand(DoubleSupplier speed) {
        return run(() -> {
            if (Overrides.MANUAL_MECH_CONTROL_MODE.getStatus()) {
                setSpeed(speed.getAsDouble());
            }
        });
    }

    /**
     * Creates a command that overrides all others and both disables and coasts the
     * motor until cancelled.
     * 
     * @return the command
     */
    /**
     * Creates a command that both disables and sets the motor to coast until
     * cancelled.
     * 
     * @return the command
     */
    public CommandBase disabledMotorOverrideCommand() {
        return runOnce(() -> motor.stopMotor())
                .andThen(() -> motor.setIdleMode(IdleMode.kCoast))
                .finallyDo((d) -> {
                    motor.setIdleMode(IdleMode.kBrake);
                    pidController.reset(getPosition());
                }).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }
}
