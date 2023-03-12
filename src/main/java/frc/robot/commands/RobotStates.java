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
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Manipulator;

public class RobotStates {
    //////////////////////////////
    ///////// RobotStates ////////
    //////////////////////////////

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

    //////////////////////////////
    ///////// Intake Mode ////////
    //////////////////////////////

    /**
     * The current mode of the robot for intaking a game piece. Set by the driver.
     */
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
     * Creates a command to set the current mode of the intake.
     * 
     * @param intakeMode the new mode to set the intake mechanism to
     * @return the command
     */
    public static CommandBase setIntakeModeCommand(Supplier<GamePiece> intakeMode) {
        return Commands.runOnce(() -> {
            setIntakeMode(intakeMode.get());
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

    public static CommandBase initializeMechanisms(
            Intake intake,
            Manipulator manipulator,
            Elevator elevator,
            Elbow elbow) {
        return Commands.sequence(
                elbow.setDesiredPositionCommand(ElbowPosition.MID, elevator),
                Commands.runOnce(elevator::disable),
                intake.setPassoversRetractedCommand(elevator),
                manipulator.setWristDownCommand(),
                elevator.zeroEncoderCommand()).withName("Initialize Mechanisms");
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
    public static CommandBase singularErrorCommand(Supplier<String> error) {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    System.out.println(error);
                    currentDiscreteError = Optional.of(error.get());
                }),
                Commands.waitSeconds(1)).finallyDo(d -> {
                    currentDiscreteError = Optional.empty();
                });
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

    //////////////////////////////
    ////////// Sequences /////////
    //////////////////////////////

    /**
     * Positive distances are towards the grid.
     * 
     * @param distance
     * @param drivetrain
     * @return
     */
    protected static CommandBase driveDeltaCommand(double distance, Drivetrain drivetrain) {
        return drivetrain.moveDeltaPathFollowingCommand(
                new Transform2d(
                        new Translation2d(distance, 0),
                        new Rotation2d(Math.PI)),
                new PathConstraints(3,
                        1))
                .withTimeout(2).withName("Drive Delta");
    }

    /**
     * Positive distances are towards the grid.
     * 
     * @param distance
     * @param drivetrain
     * @return
     */
    protected static CommandBase driveDeltaCommand(double distance, Drivetrain drivetrain,
            PathConstraints constraints) {
        return drivetrain.moveDeltaPathFollowingCommand(
                new Transform2d(
                        new Translation2d(distance, 0),
                        new Rotation2d(Math.PI)),
                constraints)
                .withTimeout(2).withName("Drive Delta");
    }

    protected static CommandBase flashButtonCommand(BooleanConsumer buttonLed) {
        return Commands.sequence(
                Commands.runOnce(() -> buttonLed.accept(true)),
                Commands.waitSeconds(0.25),
                Commands.runOnce(() -> buttonLed.accept(false)),
                Commands.waitSeconds(0.25)).repeatedly()
                .finallyDo((d) -> buttonLed.accept(false)).withName("Flash Button");
    }

    public static CommandBase runIntakingCommand(
            Supplier<GamePiece> gamePieceSupplier,
            Intake intake,
            Manipulator manipulator,
            Elevator elevator,
            Elbow elbow) {
        return Commands.sequence(
                RobotStates.setIntakeModeCommand(() -> gamePieceSupplier.get()),
                Commands.parallel(
                        elevator.setDesiredPositionCommand(ElevatorPosition.BOTTOM, intake,
                                elbow).withTimeout(0.75),
                        Commands.select(
                                Map.of(
                                        GamePiece.CONE,
                                        elbow.setDesiredPositionCommand(ElbowPosition.CONE_PICKUP, elevator),
                                        GamePiece.CUBE, elbow.setDesiredPositionCommand(ElbowPosition.MID, elevator)),
                                () -> gamePieceSupplier.get())),

                intake.setIntakeDownCommand(elevator),
                manipulator.setPincersReleasedCommand(() -> gamePieceSupplier.get()),
                Commands.parallel(
                        Commands.select(
                                Map.of(
                                        GamePiece.CONE, Commands.none(),
                                        GamePiece.CUBE, intake.setPassoversExtendedCommand(elevator)
                                                .andThen(intake.runPassoverMotorsCommand())),
                                () -> gamePieceSupplier.get()),
                        manipulator.setPincersReleasedCommand(() -> gamePieceSupplier.get())
                                .repeatedly())

        );
    }

    public static CommandBase startIntakingCommand(
            Supplier<GamePiece> gamePieceSupplier,
            Intake intake,
            Manipulator manipulator,
            Elevator elevator,
            Elbow elbow) {
        return Commands.sequence(
                RobotStates.setIntakeModeCommand(() -> gamePieceSupplier.get()),
                elevator.setDesiredPositionCommand(ElevatorPosition.BOTTOM, intake, elbow).withTimeout(0.75),
                elbow.setDesiredPositionCommand(ElbowPosition.MID, elevator),
                intake.setIntakeDownCommand(elevator),
                manipulator.setPincersReleasedCommand(() -> gamePieceSupplier.get()),
                Commands.select(
                        Map.of(
                                GamePiece.CONE, Commands.none(),
                                GamePiece.CUBE,
                                intake.setPassoversExtendedCommand(elevator)
                                        .andThen(intake.startPassoverMotorsCommand())),
                        () -> gamePieceSupplier.get()),
                manipulator.setPincersReleasedCommand(() -> gamePieceSupplier.get()));
    }

    public static CommandBase endIntakingCommand(
            Supplier<GamePiece> gamePieceSupplier,
            Intake intake,
            Manipulator manipulator,
            Elevator elevator,
            Elbow elbow) {
        return Commands.sequence(

                manipulator.setPincersPincingCommand(() -> gamePieceSupplier.get()),
                intake.setPassoversRetractedCommand(elevator),
                intake.setIntakeUpCommand(elevator),
                Commands.waitSeconds(0.2)
                        .andThen(elbow.setDesiredPositionCommand(ElbowPosition.HIGH, elevator)),
                RobotStates.setCurrentStateCommand(RobotState.SCORING),
                RobotStates.clearIntakeModeCommand());
    }

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
     * @return the command
     */
    public static CommandBase intakeGamePiece(
            BooleanSupplier secondaryButton,
            Intake intake,
            Manipulator manipulator,
            Elevator elevator,
            Elbow elbow) {
        return Commands.either(
                Commands.sequence(
                        Commands.waitUntil(secondaryButton).deadlineWith(
                                runIntakingCommand(() -> intakeMode.orElseThrow(), intake, manipulator, elevator,
                                        elbow)),
                        endIntakingCommand(() -> intakeMode.orElseThrow(), intake, manipulator, elevator,
                                elbow)),
                singularErrorCommand(() -> "Intake mode not present"),
                () -> intakeMode.isPresent()).withName("Intake Game Piece");
    }

    public static CommandBase humanPlayerExtendElevatorCommand(
            GamePiece gamePiece,
            Intake intake,
            Manipulator manipulator,
            Elevator elevator,
            Elbow elbow) {
        return Commands.sequence(
                setIntakeModeCommand(gamePiece),
                Commands.parallel(
                        elevator.setDesiredPositionCommand(ElevatorPosition.DOUBLE_SUBSTATION_PICKUP, intake, elbow),
                        elbow.setDesiredPositionCommand(ElbowPosition.DOUBLE_SUBSTATION_PICKUP, elevator)),
                manipulator.setPincersReleasedCommand(() -> gamePiece),
                Commands.waitSeconds(0.5));
    }

    public static CommandBase humanPlayerPickupCommand(
            BooleanSupplier secondaryButton,
            BooleanConsumer secondaryButtonLED,
            GamePiece gamePiece,
            Drivetrain drivetrain,
            Intake intake,
            Manipulator manipulator,
            Elevator elevator,
            Elbow elbow) {
        return Commands.sequence(
                setIntakeModeCommand(gamePiece),
                Commands.parallel(
                        elevator.setDesiredPositionCommand(ElevatorPosition.DOUBLE_SUBSTATION_PICKUP, intake, elbow),
                        elbow.setDesiredPositionCommand(ElbowPosition.DOUBLE_SUBSTATION_PICKUP, elevator)),
                manipulator.setPincersReleasedCommand(() -> gamePiece),
                Commands.waitSeconds(0.5),
                Commands.deadline(
                        Commands.waitUntil(secondaryButton::getAsBoolean),
                        flashButtonCommand(secondaryButtonLED)),
                manipulator.setPincersPincingCommand(() -> gamePiece),
                clearIntakeModeCommand())
                .withName("Score Game Piece");
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
     * @return
     */
    public static CommandBase stowElevatorCommand(
            Intake intake,
            Manipulator manipulator,
            Elevator elevator,
            Elbow elbow) {
        return Commands.sequence(
                manipulator.setWristDownCommand(),
                manipulator.setPincersClosedCommand(),
                elevator.setDesiredPositionCommand(ElevatorPosition.BOTTOM, intake, elbow),
                // .deadlineWith(elbow.lockPosition()),
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
     * @return
     */
    public static CommandBase stowElevatorHPStationCommand(
            Intake intake,
            Manipulator manipulator,
            Elevator elevator,
            Elbow elbow) {
        return Commands.sequence(
                elevator.setDesiredPositionCommand(ElevatorPosition.BOTTOM, intake, elbow),
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
    public static PathPlannerTrajectory getAutoDriveTraj(Supplier<RobotState> currentState,
            Supplier<GamePieceLocation> location,
            Drivetrain drivetrain) {
        return getAutoDriveTraj(new PathConstraints(3, 2), currentState, location, drivetrain);
    }

    public static PathPlannerTrajectory getAutoDriveTraj(PathConstraints constraints, Supplier<RobotState> currentState,
            Supplier<GamePieceLocation> location,
            Drivetrain drivetrain) {

        Pose2d targetPose;
        Map<Rectangle2d, Pose2d[]> map;
        if (currentState.get() == RobotState.SEEKING) {
            targetPose = DriverStation.getAlliance() == Alliance.Blue
                    ? FieldConstants.Blue.Substations.SINGLE_SUBSTATION
                    : FieldConstants.Red.Substations.SINGLE_SUBSTATION;

            map = FieldConstants.AutoDrive.HP_STATION_ZONE_TO_INTERMEDIARY
                    .get(DriverStation.getAlliance());

        } else if (currentState.get() == RobotState.SCORING) {
            if (location.get() == null) {
                return new PathPlannerTrajectory();
            } // so that the rest doesn't throw an exception and crash code

            targetPose = FieldConstants.scoringLocationMap.get(DriverStation.getAlliance())
                    .get(location.get());

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
        Pose2d last;
        try {
            last = intermediaryPoses[intermediaryPoses.length - 1];
        } catch (Exception e) {
            last = drivetrain.getPose();
        }
        pathPoints.add(new PathPoint(targetPose.getTranslation(),
                new Rotation2d(targetPose.getX() - last.getX(),
                        targetPose.getY() - last.getY()),
                targetPose.getRotation()));

        PathPlannerTrajectory traj = PathPlanner.generatePath(
                constraints,
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
     * @return the command
     */
    public static CommandBase autoDriveCommand(Drivetrain drivetrain, GridInterface gridInterface) {
        Supplier<Command> pathFollowingCommandSupplier = () -> drivetrain
                .pathFollowingCommand(getAutoDriveTraj(RobotStates::getCurrentState,
                        () -> gridInterface.getSetLocation().orElseGet(null), drivetrain));

        // this is a proxy command because we have to do things with the trajectory
        // every time before passing it into the `drivetrain.pathFollowingCommand`
        // method.
        return Commands.either(
                new ProxyCommand(pathFollowingCommandSupplier).finallyDo((d) -> {
                    System.out.println(d);
                    drivetrain.stop();
                }),
                RobotStates.singularErrorCommand(() -> "Grid interface location not present"),
                () -> gridInterface.getSetLocation().isPresent() || RobotStates.getCurrentState() == RobotState.SEEKING)
                .withName("Auto Drive");
    }

    /**
     * Creates a command that drives to the location specified by the operator
     * interface.
     * 
     * The command will not run if the location has not been set.
     * 
     * @param drivetrain
     * @param gridInterface
     * @return the command
     */
    public static CommandBase autoDriveCommand(
            Supplier<RobotState> state,
            Supplier<GamePieceLocation> location,
            Drivetrain drivetrain) {
        return autoDriveCommand(new PathConstraints(3, 2), state, location, drivetrain);
    }

    public static CommandBase autoDriveCommand(
            PathConstraints constraints,
            Supplier<RobotState> state,
            Supplier<GamePieceLocation> location,
            Drivetrain drivetrain) {
        Supplier<Command> pathFollowingCommandSupplier = () -> drivetrain
                .pathFollowingCommand(getAutoDriveTraj(constraints, state, location, drivetrain));

        // this is a proxy command because we have to do things with the trajectory
        // every time before passing it into the `drivetrain.pathFollowingCommand`
        // method.
        return new ProxyCommand(pathFollowingCommandSupplier)
                .finallyDo((d) -> drivetrain.stop())
                .withName("Auto Drive");
    }

}
