package frc.robot.commands;

import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.techhounds.houndutil.houndauto.AutoManager;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Manipulator;

public class RobotStates {
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
     * Clears the intake mode to a null state. Trying to set the pincers to released
     * or pincing will result in an error state.
     */
    public static void clearIntakeMode() {
        RobotStates.intakeMode = Optional.empty();
    }

    /**
     * Creates a command to "prepare" the robot for intaking
     * game pieces.
     * 
     * Runs this sequence:
     *     1. Sets the elevator to the bottom position.
     *     2. Sets the elbow to the middle position.
     *     3. Sets the pincers to the released position for the current game piece
     * mode.
     * 
     * The entire sequence will not run unless the intake mode has been set.
     * 
     * @param intake
     * @param manipulator
     * @param elevator
     * @param elbow
     * @param leds
     * @return the command
     */
    public static CommandBase prepareToIntakeGamePiece(
            Intake intake,
            Manipulator manipulator,
            Elevator elevator,
            Elbow elbow,
            LEDs leds) {
        return Commands.either(
                Commands.sequence(
                        elevator.setDesiredPositionCommand(ElevatorPosition.BOTTOM, leds),
                        elbow.setDesiredPositionCommand(ElbowPosition.MID),
                        manipulator.setPincersReleasedCommand(intakeMode.orElse(GamePiece.NONE))),
                leds.errorCommand(),
                () -> intakeMode.isPresent()).withName("prepareToIntakeGamePiece");
    }

    /**
     * Creates a command to intake a game piece.
     * 
     * Runs this sequence:
     *     1. Sets the intake down.
     *     2. Sets the passover to be extended, or able to intake game pieces.
     *     3. Runs the passover motors until the intake has detected a game piece.
     *     4. Sets the pincers to "pince" the game piece.
     *     5. Sets the passover to be retracted.
     *     6. Sets the elbow to the upper position, so that cones don't drag on the
     *     ground when the robot moves.
     *     7. Sets the intake up.
     *     8. Clears the intake mode.
     * 
     * The entire sequence will not run unless the intake mode has been set.
     * 
     * @param intake
     * @param manipulator
     * @param elevator
     * @param elbow
     * @param leds
     * @return the command
     */
    public static CommandBase intakeGamePiece(
            Intake intake,
            Manipulator manipulator,
            Elevator elevator,
            Elbow elbow,
            LEDs leds) {
        return Commands.either(
                Commands.sequence(
                        intake.setIntakeDownCommand(),
                        intake.setPassoverExtendedCommand(),
                        intake.runPassoverMotorsCommand().until(intake::isGamePieceDetected),
                        manipulator.setPincersPincingCommand(intakeMode.orElse(GamePiece.NONE)),
                        intake.setPassoverRetractedCommand(),
                        elbow.setDesiredPositionCommand(ElbowPosition.HIGH),
                        intake.setIntakeUpCommand(),
                        Commands.runOnce(RobotStates::clearIntakeMode)),
                leds.errorCommand(),
                () -> intakeMode.isPresent()).withName("intakeGamePiece");
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
                                        gridInterface.getSetLocation()
                                                .orElse(GamePieceLocation.NONE).gamePiece,
                                        gridInterface.getSetLocation().orElse(GamePieceLocation.NONE).level, leds),
                                Commands.sequence(
                                        elbow.setDesiredPositionCommand(ElbowPosition.MID),
                                        Commands.select(
                                                Map.of(
                                                        GamePiece.CONE,
                                                        Commands.waitSeconds(0.5)
                                                                .andThen(elbow
                                                                        .setDesiredPositionCommand(ElbowPosition.HIGH))
                                                                .alongWith(manipulator.setWristUpCommand()),

                                                        GamePiece.CUBE,
                                                        Commands.none()),
                                                () -> gridInterface.getSetLocation()
                                                        .orElse(GamePieceLocation.NONE).gamePiece))),
                        Commands.select(
                                Map.of(
                                        GamePiece.CONE,
                                        Commands.waitUntil(manipulator::isPoleDetected)
                                                .andThen(manipulator
                                                        .setPincersReleasedCommand(
                                                                gridInterface.getSetLocation()
                                                                        .orElse(GamePieceLocation.NONE).gamePiece)
                                                        .alongWith(elbow.setDesiredPositionCommand(ElbowPosition.LOW))),

                                        GamePiece.CUBE,
                                        Commands.waitUntil(elevator::isAtGoal)
                                                .andThen(manipulator
                                                        .setPincersReleasedCommand(
                                                                gridInterface.getSetLocation()
                                                                        .orElse(GamePieceLocation.NONE).gamePiece))),
                                () -> gridInterface.getSetLocation()
                                        .orElse(GamePieceLocation.NONE).gamePiece)),
                leds.errorCommand(),
                () -> gridInterface.getSetLocation().isPresent()).withName("scoreGamePiece");
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
                elbow.setDesiredPositionCommand(ElbowPosition.MID),
                elevator.setDesiredPositionCommand(ElevatorPosition.BOTTOM, leds),
                Commands.waitUntil(elevator::isAtGoal)).withName("stowElevator");
    }

    /**
     * Creates a trajectory to the specified game piece location based on the
     * drivetrain's current position, and displays it on the AutoManager's Field2d.
     * 
     * @param drivetrain
     * @param gridInterface
     * @return the trajectory
     */
    public static PathPlannerTrajectory getDriveToScoringLocationTraj(Drivetrain drivetrain,
            GridInterface gridInterface) {

        if (gridInterface.getSetLocation().isEmpty()) {
            return new PathPlannerTrajectory();
        } // so that the rest doesn't throw an exception and crash code
        GamePieceLocation gamePieceLocation = gridInterface.getSetLocation().orElseThrow();
        Pose2d targetPose = FieldConstants.scoringLocationMap.get(DriverStation.getAlliance())
                .get(gamePieceLocation);

        PathPlannerTrajectory traj = PathPlanner.generatePath(
                new PathConstraints(4, 3),
                new PathPoint(drivetrain.getPose().getTranslation(),
                        new Rotation2d(targetPose.getX() - drivetrain.getPose().getX(), // in order to
                                                                                        // angle towards
                                                                                        // target
                                targetPose.getY() - drivetrain.getPose().getY()),
                        drivetrain.getPose().getRotation()),
                new PathPoint(targetPose.getTranslation(), targetPose.getRotation()));

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
    public static CommandBase driveToScoringLocation(Drivetrain drivetrain, GridInterface gridInterface, LEDs leds) {
        Supplier<Command> pathFollowingCommandSupplier = () -> drivetrain
                .pathFollowingCommand(getDriveToScoringLocationTraj(drivetrain, gridInterface));

        // this is a proxy command because we have to do things with the trajectory
        // every time before passing it into the `drivetrain.pathFollowingCommand`
        // method.
        return Commands.either(
                new ProxyCommand(pathFollowingCommandSupplier)
                        .finallyDo((d) -> drivetrain.stop()),
                leds.errorCommand(),
                () -> gridInterface.getSetLocation().isPresent()).withName("driveToScoringLocation");
    }
}
