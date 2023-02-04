package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.techhounds.houndutil.houndlib.Utils;
import com.techhounds.houndutil.houndlog.LogGroup;
import com.techhounds.houndutil.houndlog.LogProfileBuilder;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.loggers.DeviceLogger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;

/**
 * Elbow subsystem, one CAN Motor and encoder, one feedforward object, and two
 * hall effect sensors.
 * 
 * @author jc
 */
public class Elbow extends ProfiledPIDSubsystem {
    public static enum ElbowPosition {
        LOW(0),
        MID(0),
        HIGH(0);

        public final int value;

        private ElbowPosition(int value) {
            this.value = value;
        }
    }

    /**
     * The motor that controls the elbow.
     */
    private CANSparkMax motor = new CANSparkMax(Constants.Elbow.CANIDs.ELBOW_MOTOR, MotorType.kBrushless);

    /**
     * The encoder (connected to the Spark MAX) that detects the angle of the elbow.
     */
    private SparkMaxAbsoluteEncoder encoder = motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

    /**
     * The feedforward controller (calculates voltage based on the setpoint's
     * velocity and acceleration).
     */
    private ArmFeedforward feedforwardController = new ArmFeedforward(Constants.Elbow.Gains.kS.get(),
            Constants.Elbow.Gains.kG.get(), Constants.Elbow.Gains.kV.get(), Constants.Elbow.Gains.kA.get());

    /**
     * Hall effect sensors in the motor that will limit freedom of movement.
     */
    private DigitalInput bottomHallEffect = new DigitalInput(Constants.Elbow.BOTTOM_HALL_EFFECT_PORT);
    private DigitalInput topHallEffect = new DigitalInput(Constants.Elbow.TOP_HALL_EFFECT_PORT);

    /**
     * Initializes the elbow.
     */
    public Elbow() {
        super(new ProfiledPIDController(
                Constants.Elbow.Gains.kP.get(),
                Constants.Elbow.Gains.kP.get(),
                Constants.Elbow.Gains.kP.get(),
                new TrapezoidProfile.Constraints(
                        Constants.Elbow.MAX_VELOCITY_METERS_PER_SECOND.get(),
                        Constants.Elbow.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED.get())));

        getController().setTolerance(Constants.Elbow.Gains.TOLERANCE.get());

        LoggingManager.getInstance().addGroup("Elbow", new LogGroup(
                new DeviceLogger<CANSparkMax>(motor, "Elbow Motor",
                        LogProfileBuilder.buildCANSparkMaxLogItems(motor))));
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
        return encoder.getPosition();
    }

    /**
     * Sets the speed of the motor. Will refuse to go past the bottom or top points
     * of the elbow.
     * 
     * @param speed the speed, from -1.0 to 1.0
     */
    public void setSpeed(double speed) {
        if (!Utils.limitMechanism(bottomHallEffect.get(), topHallEffect.get(), speed))
            motor.set(speed);
        else
            motor.stopMotor();
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
        else
            motor.stopMotor();
    }

    /**
     * Returns an InstantCommand that sets the goal position of the elbow, and
     * enables the controller.
     * 
     * @param position an ElbowPosition to set the elbow to
     * @return the command
     */
    public CommandBase setDesiredPositionCommand(ElbowPosition position) {
        return runOnce(() -> setGoal(position.value)).andThen(this::enable);
    }
}
