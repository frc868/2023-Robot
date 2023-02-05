package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import com.techhounds.houndutil.houndlib.Utils;
import com.techhounds.houndutil.houndlog.LogGroup;
import com.techhounds.houndutil.houndlog.LogProfileBuilder;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.loggers.DeviceLogger;
import com.techhounds.houndutil.houndlog.loggers.Logger;
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

        public final int value;

        private ElevatorPosition(int value) {
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
            Constants.Elevator.Gains.kS.get(),
            Constants.Elevator.Gains.kG.get(),
            Constants.Elevator.Gains.kV.get(),
            Constants.Elevator.Gains.kA.get());

    /**
     * Used to make sure that the PID controller does not enable unless the elevator
     * has been zeroed off of the hall effects.
     */
    private boolean isInitialized = false;

    /**
     * Initializes the elevator.
     */
    public Elevator() {
        super(new ProfiledPIDController(
                Constants.Elevator.Gains.kP.get(),
                Constants.Elevator.Gains.kI.get(),
                Constants.Elevator.Gains.kD.get(),
                new TrapezoidProfile.Constraints(
                        Constants.Elevator.MAX_VELOCITY_METERS_PER_SECOND.get(),
                        Constants.Elevator.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED.get())));

        LoggingManager.getInstance().addGroup("Elevator", new LogGroup(
                new Logger[] {
                        new DeviceLogger<CANSparkMax>(leftMotor, "Primary Elevator Motor",
                                LogProfileBuilder.buildCANSparkMaxLogItems(leftMotor)),
                        new DeviceLogger<CANSparkMax>(rightMotor, "Secondary Elevator Motor",
                                LogProfileBuilder.buildCANSparkMaxLogItems(rightMotor))
                }));
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
        return leftMotor.getEncoder().getPosition();
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
    public CommandBase setDesiredPositionCommand(ElevatorPosition position, LEDs leds) {
        return Commands.either(runOnce(() -> setGoal(position.value)).andThen(this::enable), leds.errorCommand(),
                () -> isInitialized);
    }

    public Command setScoringPositionCommand(GamePiece gamePieceMode,
            Level scoringMode, LEDs leds) {
        return Commands.select(
                Map.of(
                        GamePiece.CONE,
                        Commands.select(
                                Map.of(
                                        Level.LOW,
                                        setDesiredPositionCommand(ElevatorPosition.CONE_LOW, leds),
                                        Level.MIDDLE,
                                        setDesiredPositionCommand(ElevatorPosition.CONE_MID, leds),
                                        Level.HIGH,
                                        setDesiredPositionCommand(ElevatorPosition.CONE_HIGH, leds)),
                                () -> scoringMode),
                        GamePiece.CUBE,
                        Commands.select(
                                Map.of(
                                        Level.LOW,
                                        setDesiredPositionCommand(ElevatorPosition.CUBE_LOW, leds),
                                        Level.MIDDLE,
                                        setDesiredPositionCommand(ElevatorPosition.CUBE_MID, leds),
                                        Level.HIGH,
                                        setDesiredPositionCommand(ElevatorPosition.CUBE_HIGH, leds)),
                                () -> scoringMode)),
                () -> gamePieceMode);
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
}
