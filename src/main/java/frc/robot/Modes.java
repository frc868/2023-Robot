package frc.robot;

import java.util.Optional;
import java.util.function.Supplier;

import com.techhounds.houndutil.houndlog.interfaces.Log;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Elbow.ElbowPosition;
import frc.robot.GamePieceLocation.GamePiece;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Manipulator;

public class Modes {

    @Log(groups = "states")
    private static Supplier<Boolean> initialized = Modes::isInitialized;

    @Log(groups = "states")
    private static Supplier<String> discreteError = () -> Modes.getCurrentDiscreteError().isPresent()
            ? Modes.getCurrentDiscreteError().orElseThrow().toString()
            : "None";

    @Log(groups = "states")
    private static Supplier<String> continuousError = () -> Modes.getCurrentContinuousError().isPresent()
            ? Modes.getCurrentContinuousError().orElseThrow().toString()
            : "None";

    //////////////////////////////
    ///////// RobotState /////////
    //////////////////////////////

    public enum RobotState {
        SEEKING,
        INTAKING,
        SCORING;
    }

    // RobotState
    @Log
    private static RobotState robotState = RobotState.SEEKING;

    public static RobotState getRobotState() {
        return robotState;
    }

    public static void setRobotState(RobotState robotState) {
        Modes.robotState = robotState;
    }

    public static Command setRobotStateCommand(Supplier<RobotState> robotState) {
        return Commands.runOnce(() -> Modes.robotState = robotState.get()).withName("Set Robot State");
    }

    //////////////////////////////
    ///////// Intake Mode ////////
    //////////////////////////////

    /**
     * The current mode of the robot for intaking a game piece. Set by the driver.
     */
    @Log
    private static Optional<GamePiece> intakeMode = Optional.empty();

    /**
     * Get the current mode of the intake. Note that this is an Optional, so you
     * must call {@code intakeMode.get()} to get the actual value.
     * 
     * 
     * @return the Optional containing the intake mode
     */
    public static Optional<GamePiece> getIntakeMode() {
        return intakeMode;
    }

    /**
     * Set the current mode of the intake.
     * 
     * @param intakeMode the new mode to set the intake mechanism to
     */
    public static void setIntakeMode(GamePiece intakeMode) {
        Modes.intakeMode = Optional.of(intakeMode);
    }

    /**
     * Creates a command to set the current mode of the intake.
     * 
     * @param intakeMode the new mode to set the intake mechanism to
     * @return the command
     */
    public static Command setIntakeModeCommand(Supplier<GamePiece> intakeMode) {
        return Commands.runOnce(() -> {
            setIntakeMode(intakeMode.get());
        }).withName("Set Intake Mode");
    }

    /**
     * Clears the intake mode to a null state. Trying to set the pincers to released
     * or pincing will result in an error state.
     */
    public static void clearIntakeMode() {
        Modes.intakeMode = Optional.empty();
    }

    /**
     * Creates a command to clear the intake mode.
     * 
     * @return the command
     */
    public static Command clearIntakeModeCommand() {
        return Commands.runOnce(Modes::clearIntakeMode).withName("Clear Intake Mode");
    }

    //////////////////////////////
    /////// Initialization ///////
    //////////////////////////////

    /**
     * Used to make sure that no mechanisms move until the elevator has been
     * zeroed.
     */
    private static boolean isInitialized = false;

    public static void enableInitialized() {
        isInitialized = true;
    }

    public static boolean isInitialized() {
        return isInitialized;
    }

    public static Command initializeMechanisms(
            Intake intake,
            Manipulator manipulator,
            Elevator elevator,
            Elbow elbow) {
        return Commands.sequence(
                elbow.moveToPositionCommand(() -> ElbowPosition.MID),
                intake.setPassoversRetractedCommand(elevator),
                manipulator.setWristDownCommand(),
                elevator.autoHallResetPositionCommand()).withName("Initialize Mechanisms");
    }

    //////////////////////////////
    //////// Error States ////////
    //////////////////////////////

    private static Optional<String> currentDiscreteError = Optional.empty();
    private static Optional<String> currentContinuousError = Optional.empty();

    /**
     * Creates a command that will set the current error for 2 seconds,
     * then remove the error (useful for indicating an error
     * after a button press, like if a certain value isn't set properly or a safety
     * has been triggered).
     * 
     * @return the command
     */
    public static Command singularErrorCommand(Supplier<String> error) {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    System.out.println(error);
                    currentDiscreteError = Optional.of(error.get());
                }),
                Commands.waitSeconds(0.5)).finallyDo(d -> {
                    currentDiscreteError = Optional.empty();
                });
    }

    /**
     * Creates a command that will set the error mode while the command
     * is being run, then remove the error after the command is cancelled.
     * 
     * @return the command
     */
    public static Command continuousErrorCommand(Supplier<String> error) {
        return Commands.runEnd(
                () -> {
                    currentContinuousError = Optional.of(error.get());
                },
                () -> {
                    currentContinuousError = Optional.empty();
                });
    }

    public static Optional<String> getCurrentDiscreteError() {
        return currentDiscreteError;
    }

    public static Optional<String> getCurrentContinuousError() {
        return currentContinuousError;
    }
}
