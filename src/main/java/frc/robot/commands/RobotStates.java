package frc.robot.commands;

import java.util.ArrayList;
import java.util.Map;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.techhounds.houndutil.houndauto.AutoManager;
import com.techhounds.houndutil.houndlib.Rectangle2d;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.FieldConstants;
import frc.robot.GamePieceLocation;
import frc.robot.GamePieceLocation.GamePiece;
import frc.robot.GridInterface;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Elbow.ElbowPosition;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorPosition;
import frc.robot.subsystems.LEDs.LEDState;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Manipulator;

public class RobotStates {
    public enum RobotState {
        SEEKING,
        SCORING;
    }

    // RobotState
    private static RobotState currentState = RobotState.SEEKING;

    public static RobotState getCurrentState() {
        return currentState;
    }

    public static void setCurrentState(RobotState robotState) {
        currentState = robotState;
    }

    public static CommandBase setCurrentStateCommand(RobotState robotState) {
        return Commands.runOnce(() -> currentState = robotState);
    }

    // Intake Mode

    /**
     * The current mode of the robot for intaking a game piece. Set by the driver.
     */
    private static Optional<GamePiece> intakeMode = Optional.empty();

    /**
     * Get the current mode of the intake. Note that this is an Optional, so you
     * must call {@code intakeMode.get()} to get the actual value.
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
        RobotStates.intakeMode = Optional.of(intakeMode);
    }

    /**
     * Creates a command to set the current mode of the intake.
     * 
     * @param intakeMode the new mode to set the intake mechanism to
     * @return the command
     */
    public static CommandBase setIntakeModeCommand(GamePiece intakeMode) {
        return Commands.runOnce(() -> {
            setIntakeMode(intakeMode);
        });
    }

    /**
     * Clears the intake mode to a null state. Trying to set the pincers to released
     * or pincing will result in an error state.
     */
    public static void clearIntakeMode() {
        RobotStates.intakeMode = Optional.empty();
    }

    /**
     * Creates a command to clear the intake mode.
     * 
     * @return the command
     */
    public static CommandBase clearIntakeModeCommand() {
        return Commands.runOnce(RobotStates::clearIntakeMode);
    }

    // Initialization

    /**
     * Used to make sure that no mechanisms move until the elevator has been
     * zeroed.
     */
    private static boolean isInitialized = true;

    public static void enableInitialized() {
        isInitialized = true;
    }

    public static boolean isInitialized() {
        return isInitialized;
    }

    public static CommandBase initializeMechanisms(
            Intake intake,
            Manipulator manipulator,
            Elevator elevator,
            Elbow elbow,
            LEDs leds) {
        return Commands.sequence(
                intake.setPassoversRetractedCommand(elevator, leds),
                manipulator.setWristDownCommand(),
                elbow.setDesiredPositionCommand(ElbowPosition.MID, elevator),
                elevator.zeroEncoderCommand(),
                leds.setLEDStateCommand(LEDState.TechHOUNDS));
    }

    // Error States

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
    public static CommandBase singularErrorCommand(Supplier<String> error) {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    System.out.println(error);
                    currentDiscreteError = Optional.of(error.get());
                }),
                Commands.waitSeconds(2),
                Commands.runOnce(() -> {
                    currentDiscreteError = Optional.empty();
                }));
    }

    /**
     * Creates a command that will set the error mode while the command
     * is being run, then remove the error after the command is cancelled.
     * 
     * @return the command
     */
    public static CommandBase continuousErrorCommand(Supplier<String> error) {
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

    // Sequences

    /**
     * Creates a command to intake a game piece.
     * 
     * Runs this sequence:
     *     1. Sets the intake down.
     *     2. Sets the passovers to the extended position.
     *     3. Sets the passover to be extended, or able to intake game pieces.
     *     4. Runs the passover motors until the intake has detected a game piece,
     *        and continuously checks the intake mode for an update.
     *     5. Waits for a button press from the driver.
     *     4. Sets the pincers to "pince" the game piece.
     *     5. Sets the passover to be retracted.
     *     6. Sets the elbow to the upper position, so that cones don't drag on the
     *     ground when the robot moves.
     *     7. Sets the intake up.
     *     8. Clears the intake mode.
     * 
     * The entire sequence will not run unless the intake mode has been set.
     * 
     * @param secondaryButton a reference to the second button that a driver has to
     *                        press to continue the sequence
     * @param intake
     * @param manipulator
     * @param elevator
     * @param elbow
     * @param leds
     * @return the command
     */
    public static CommandBase intakeGamePiece(
            BooleanSupplier secondaryButton,
            Intake intake,
            Manipulator manipulator,
            Elevator elevator,
            Elbow elbow,
            LEDs leds) {
        return Commands.either(
                Commands.sequence(
                        elevator.setDesiredPositionCommand(ElevatorPosition.BOTTOM, intake, elbow, leds),
                        elbow.setDesiredPositionCommand(ElbowPosition.MID, elevator),
                        intake.setIntakeDownCommand(elevator, leds),
                        manipulator.setPincersReleasedCommand(() -> intakeMode.orElseThrow()),
                        intake.setPassoversExtendedCommand(elevator, leds),
                        Commands.deadline(
                                Commands.waitUntil(secondaryButton::getAsBoolean),
                                intake.runPassoverMotorsCommand(),
                                manipulator.setPincersReleasedCommand(() -> intakeMode.orElseThrow()).repeatedly()),
                        // Commands.waitUntil(),
                        manipulator.setPincersPincingCommand(() -> intakeMode.orElseThrow()),
                        intake.setPassoversRetractedCommand(elevator, leds),
                        intake.setIntakeUpCommand(elevator, leds),
                        RobotStates.setCurrentStateCommand(RobotState.SCORING),
                        elbow.setDesiredPositionCommand(ElbowPosition.HIGH, elevator),
                        RobotStates.clearIntakeModeCommand()),
                singularErrorCommand(() -> "Intake mode not present"),
                () -> intakeMode.isPresent()).withName("Intake Game Piece");
    }

    /**
     * Creates a command to score a game piece, using
     * everything but the drivetrain.
     * 
     * Runs this sequence:
     *     1. Runs these two items in parallel:
     *         1. Starts moving the elevator up to the position set by the operator.
     *         2. Runs this sequence:
     *             1. Sets the elbow to the middle position.
     *             2. If the robot is set to the cone mode:
     *                 1. Waits 0.5 seconds.
     *                 2. Sets the elbow to the high position.
     *                 3. Sets the wrist up.
     *             3. If the robot is set to the cube mode:
     *                 1. Does nothing.
     *     2. If the robot is set to the cone mode:
     *         1. Waits until the pole is detected.
     *         2. Sets the pincers to release the cone.
     *         3. Sets the elbow to the low position.
     *     3. If the robot is set to the cube mode:
     *         1. Waits until the elevator reaches its goal position.
     *         2. Sets the pincers to release the cube, dropping it onto the box.
     * 
     * The entire sequence will not run unless the scoring position has been set by
     * the operator.
     * 
     * @param gridInterface
     * @param intake
     * @param manipulator
     * @param elevator
     * @param elbow
     * @param leds
     * @return the command
     */
    public static CommandBase scoreGamePiece(
            BooleanSupplier secondaryButton,
            BooleanConsumer secondaryButtonLED,
            GridInterface gridInterface,
            Intake intake,
            Manipulator manipulator,
            Elevator elevator,
            Elbow elbow,
            LEDs leds) {
        return Commands.either(
                Commands.sequence(
                        Commands.parallel(
                                elevator.setScoringPositionCommand(
                                        () -> gridInterface.getSetLocation()
                                                .orElseThrow().gamePiece,
                                        () -> gridInterface.getSetLocation().orElseThrow().level, intake, elbow, leds),
                                Commands.sequence(
                                        elbow.setDesiredPositionCommand(ElbowPosition.MID, elevator),
                                        Commands.select(
                                                Map.of(
                                                        GamePiece.CONE,
                                                        Commands.waitSeconds(2)
                                                                .andThen(elbow.setDesiredPositionCommand(
                                                                        ElbowPosition.HIGH,
                                                                        elevator).andThen(
                                                                                manipulator.setWristUpCommand(elevator,
                                                                                        leds))),

                                                        GamePiece.CUBE,
                                                        Commands.waitSeconds(2)
                                                                .andThen(elbow.setDesiredPositionCommand(
                                                                        ElbowPosition.HIGH,
                                                                        elevator))),
                                                () -> gridInterface.getSetLocation()
                                                        .orElseThrow().gamePiece))),
                        Commands.select(
                                Map.of(
                                        GamePiece.CONE,
                                        Commands.waitUntil(
                                                () -> manipulator.isPoleDetected() || secondaryButton.getAsBoolean())
                                                .alongWith(Commands.sequence(
                                                        Commands.runOnce(() -> secondaryButtonLED.accept(true)),
                                                        Commands.waitSeconds(0.5),
                                                        Commands.runOnce(() -> secondaryButtonLED.accept(false)),
                                                        Commands.waitSeconds(0.5)).repeatedly()
                                                        .finallyDo((d) -> secondaryButtonLED.accept(false)))
                                                .andThen(
                                                        Commands.parallel(
                                                                manipulator
                                                                        .setPincersReleasedCommand(
                                                                                () -> gridInterface.getSetLocation()
                                                                                        .orElseThrow().gamePiece),
                                                                elbow.setDesiredPositionCommand(ElbowPosition.LOW,
                                                                        elevator),
                                                                elevator.setDesiredPositionDeltaCommand(-0.2, intake,
                                                                        elbow, leds))
                                                                .andThen(elbow.setDesiredPositionCommand(
                                                                        ElbowPosition.HIGH, elevator))),

                                        GamePiece.CUBE,
                                        Commands.waitUntil(secondaryButton::getAsBoolean).alongWith(Commands.sequence(
                                                Commands.runOnce(() -> secondaryButtonLED.accept(true)),
                                                Commands.waitSeconds(0.5),
                                                Commands.runOnce(() -> secondaryButtonLED.accept(false)),
                                                Commands.waitSeconds(0.5)).repeatedly()
                                                .finallyDo((d) -> secondaryButtonLED.accept(false)))
                                                .andThen(manipulator
                                                        .setPincersReleasedCommand(
                                                                () -> gridInterface.getSetLocation()
                                                                        .orElseThrow().gamePiece))),
                                () -> gridInterface.getSetLocation()
                                        .orElseThrow().gamePiece),
                        Commands.waitSeconds(0.5)),
                singularErrorCommand(() -> "Grid interface location not present"),
                () -> gridInterface.getSetLocation().isPresent()).withName("Score Game Piece");
    }

    public static CommandBase humanPlayerPickup(
            GamePiece gamePiece,
            GridInterface gridInterface,
            Intake intake,
            Manipulator manipulator,
            Elevator elevator,
            Elbow elbow,
            LEDs leds) {
        return Commands.sequence(
                Commands.parallel(
                        elevator.setDesiredPositionCommand(ElevatorPosition.CUBE_MID, intake, elbow, leds),
                        Commands.sequence(
                                elbow.setDesiredPositionCommand(ElbowPosition.MID, elevator),
                                Commands.waitSeconds(1.5),
                                elbow.setDesiredPositionCommand(ElbowPosition.HIGH,
                                        elevator))),
                manipulator.setPincersReleasedCommand(() -> gamePiece),
                Commands.waitSeconds(0.5)).withName("Score Game Piece");
    }

    /**
     * Creates a command to stow the elevator after scoring a game piece.
     * 
     * Runs this sequence:
     *     1. Sets the wrist to the down position.
     *     2. Sets the elbow to the middle position.
     *     3. Moves the elevator to the bottom position, returns when the goal has
     * been reached.
     * 
     * The entire sequence will not run unless the intake mode has been set.
     * 
     * @param intake
     * @param manipulator
     * @param elevator
     * @param elbow
     * @param leds
     * @return
     */
    public static CommandBase stowElevator(
            Intake intake,
            Manipulator manipulator,
            Elevator elevator,
            Elbow elbow,
            LEDs leds) {
        return Commands.sequence(
                manipulator.setWristDownCommand(),
                manipulator.setPincersClosedCommand(),
                elbow.setDesiredPositionCommand(ElbowPosition.MID, elevator),
                elevator.setDesiredPositionCommand(ElevatorPosition.BOTTOM, intake, elbow, leds),
                RobotStates.setCurrentStateCommand(RobotState.SEEKING))
                .withName("Stow Elevator");
    }

    /**
     * Creates a command to stow the elevator after scoring a game piece.
     * 
     * Runs this sequence:
     *     1. Sets the wrist to the down position.
     *     2. Sets the elbow to the middle position.
     *     3. Moves the elevator to the bottom position, returns when the goal has
     * been reached.
     * 
     * The entire sequence will not run unless the intake mode has been set.
     * 
     * @param intake
     * @param manipulator
     * @param elevator
     * @param elbow
     * @param leds
     * @return
     */
    public static CommandBase stowElevatorHPStation(
            Intake intake,
            Manipulator manipulator,
            Elevator elevator,
            Elbow elbow,
            LEDs leds) {
        return Commands.sequence(
                manipulator.setPincersPincingCommand(() -> manipulator.getPincers() ? GamePiece.CONE : GamePiece.CUBE),
                elbow.setDesiredPositionCommand(ElbowPosition.MID, elevator),
                elevator.setDesiredPositionCommand(ElevatorPosition.BOTTOM, intake, elbow, leds),
                elbow.setDesiredPositionCommand(ElbowPosition.HIGH, elevator),
                RobotStates.setCurrentStateCommand(RobotState.SCORING))
                .withName("Stow Elevator");
    }

    /**
     * Creates a trajectory to the specified game piece location based on the
     * drivetrain's current position, and displays it on the AutoManager's Field2d.
     * 
     * @param drivetrain
     * @param gridInterface
     * @return the trajectory
     */
    public static PathPlannerTrajectory getAutoDriveTraj(Drivetrain drivetrain,
            GridInterface gridInterface) {

        Pose2d targetPose;
        Map<Rectangle2d, Pose2d[]> map;
        if (getCurrentState() == RobotState.SEEKING) {
            targetPose = DriverStation.getAlliance() == Alliance.Blue
                    ? FieldConstants.Blue.Substations.SINGLE_SUBSTATION
                    : FieldConstants.Red.Substations.SINGLE_SUBSTATION;

            map = FieldConstants.AutoDrive.HP_STATION_ZONE_TO_INTERMEDIARY
                    .get(DriverStation.getAlliance());

        } else if (getCurrentState() == RobotState.SCORING) {
            if (gridInterface.getSetLocation().isEmpty()) {
                return new PathPlannerTrajectory();
            } // so that the rest doesn't throw an exception and crash code

            GamePieceLocation gamePieceLocation = gridInterface.getSetLocation().orElseThrow();
            targetPose = FieldConstants.scoringLocationMap.get(DriverStation.getAlliance())
                    .get(gamePieceLocation);

            map = FieldConstants.AutoDrive.SCORING_AREA_ZONE_TO_INTERMEDIARY
                    .get(DriverStation.getAlliance());

        } else {
            return new PathPlannerTrajectory();
        }

        Pose2d[] intermediaryPoses = null;
        for (Rectangle2d rect : map.keySet()) {
            if (rect.isInRect(drivetrain.getPose())) {
                intermediaryPoses = map.get(rect);
            }
        }

        ArrayList<PathPoint> pathPoints = new ArrayList<PathPoint>();

        // in order to angle towards target

        try {
            pathPoints.add(
                    new PathPoint(
                            drivetrain.getPose().getTranslation(),
                            new Rotation2d(intermediaryPoses[0].getX() - drivetrain.getPose().getX(),
                                    intermediaryPoses[0].getY() - drivetrain.getPose().getY()),
                            drivetrain.getPose().getRotation()));
            for (Pose2d pose : intermediaryPoses) {
                pathPoints.add(new PathPoint(pose.getTranslation(), pose.getRotation(), pose.getRotation()));
            }
        } catch (NullPointerException e) {
            pathPoints.add(
                    new PathPoint(
                            drivetrain.getPose().getTranslation(),
                            new Rotation2d(targetPose.getX() - drivetrain.getPose().getX(),
                                    targetPose.getY() - drivetrain.getPose().getY()),
                            drivetrain.getPose().getRotation()));
        }

        pathPoints.add(new PathPoint(targetPose.getTranslation(), targetPose.getRotation(), targetPose.getRotation()));

        PathPlannerTrajectory traj = PathPlanner.generatePath(
                new PathConstraints(6, 8),
                pathPoints);

        AutoManager.getInstance().getField().getObject("AutoDrive Trajectory").setTrajectory(traj);
        return traj;
    }

    /**
     * Creates a command that drives to the location specified by the operator
     * interface.
     * 
     * The command will not run if the location has not been set.
     * 
     * @param drivetrain
     * @param gridInterface
     * @param leds
     * @return the command
     */
    public static CommandBase autoDrive(Drivetrain drivetrain, GridInterface gridInterface, LEDs leds) {
        Supplier<Command> pathFollowingCommandSupplier = () -> drivetrain
                .pathFollowingCommand(getAutoDriveTraj(drivetrain, gridInterface));

        // this is a proxy command because we have to do things with the trajectory
        // every time before passing it into the `drivetrain.pathFollowingCommand`
        // method.
        return Commands.either(
                new ProxyCommand(pathFollowingCommandSupplier).finallyDo((d) -> drivetrain.stop()),
                singularErrorCommand(() -> "Grid interface location not present"),
                () -> gridInterface.getSetLocation().isPresent() || getCurrentState() == RobotState.SEEKING)
                .withName("Drive To Scoring Location");
    }
}
