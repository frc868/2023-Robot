package frc.robot.commands;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.GamePieceLocation.GamePiece;
import frc.robot.GamePieceLocation.Level;
import frc.robot.GridInterface;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Elbow.ElbowPosition;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Manipulator;

public class Scoring {

    private static CommandBase raiseElevatorCommand(
            Supplier<GamePiece> gamePieceSupplier,
            Supplier<Level> levelSupplier,
            Intake intake,
            Manipulator manipulator,
            Elevator elevator,
            Elbow elbow) {
        return Commands.parallel(
                elevator.setScoringPositionCommand(
                        gamePieceSupplier,
                        levelSupplier, intake, elbow),
                Commands.sequence(
                        elbow.setDesiredPositionCommand(ElbowPosition.MID, elevator),
                        Commands.select(
                                Map.of(
                                        GamePiece.CONE,
                                        Commands.waitSeconds(0.5)
                                                .andThen(elbow.setDesiredPositionCommand(
                                                        ElbowPosition.HIGH,
                                                        elevator).andThen(
                                                                manipulator.setWristUpCommand(elevator))),

                                        GamePiece.CUBE,
                                        Commands.waitSeconds(0.5)
                                                .andThen(elbow.setDesiredPositionCommand(
                                                        ElbowPosition.HIGH,
                                                        elevator))),
                                gamePieceSupplier::get)));
    }

    /**
     * Positve distances are towards the grid.
     * 
     * @param distance
     * @param drivetrain
     * @return
     */
    private static CommandBase driveDeltaCommand(double distance, Drivetrain drivetrain) {
        return drivetrain.moveDeltaPathFollowingCommand(
                new Transform2d(
                        new Translation2d(distance, 0),
                        new Rotation2d(Math.PI)),
                new PathConstraints(3,
                        1))
                .withTimeout(2);
    }

    private static CommandBase placePieceCommand(
            boolean driveBackwards,
            BooleanSupplier secondaryButton,
            BooleanConsumer secondaryButtonLED,
            Supplier<GamePiece> gamePieceSupplier,
            Supplier<Level> levelSupplier,
            Drivetrain drivetrain,
            Intake intake,
            Manipulator manipulator,
            Elevator elevator,
            Elbow elbow) {
        return Commands.sequence(
                Commands.select(
                        Map.of(
                                GamePiece.CONE,
                                Commands.deadline(
                                        Commands.waitUntil(
                                                () -> (manipulator.isPoleDetected() || secondaryButton.getAsBoolean())))
                                        .andThen(
                                                Commands.sequence(
                                                        Commands.parallel(
                                                                manipulator
                                                                        .setPincersReleasedCommand(gamePieceSupplier),
                                                                elbow.setDesiredPositionCommand(ElbowPosition.LOW,
                                                                        elevator),
                                                                elevator.setDesiredPositionDeltaCommand(-0.2, intake,
                                                                        elbow),
                                                                Commands.either(
                                                                        driveDeltaCommand(-0.095, drivetrain),
                                                                        Commands.none(),
                                                                        () -> driveBackwards)),

                                                        Commands.either(
                                                                Commands.parallel(
                                                                        manipulator.setWristDownCommand(),
                                                                        elbow.setDesiredPositionCommand(
                                                                                ElbowPosition.HIGH, elevator)
                                                                                .withTimeout(.5),
                                                                        driveDeltaCommand(-0.4,
                                                                                drivetrain)),
                                                                Commands.none(),
                                                                () -> driveBackwards
                                                                        && levelSupplier.get() == Level.HIGH
                                                                        && gamePieceSupplier.get() == GamePiece.CONE))),
                                GamePiece.CUBE,
                                Commands.deadline(
                                        Commands.waitUntil(() -> secondaryButton.getAsBoolean()),
                                        Commands.sequence(
                                                Commands.runOnce(() -> secondaryButtonLED.accept(true)),
                                                Commands.waitSeconds(0.5),
                                                Commands.runOnce(() -> secondaryButtonLED.accept(false)),
                                                Commands.waitSeconds(0.5)).repeatedly()
                                                .finallyDo((d) -> secondaryButtonLED.accept(false)))
                                        .andThen(Commands.sequence(
                                                manipulator.setPincersReleasedCommand(gamePieceSupplier),
                                                Commands.either(
                                                        driveDeltaCommand(-0.25, drivetrain),
                                                        Commands.none(),
                                                        () -> driveBackwards)))),
                        gamePieceSupplier::get));
    }

    private static CommandBase placePieceAutoCommand(
            Supplier<GamePiece> gamePieceSupplier,
            Supplier<Level> levelSupplier,
            Drivetrain drivetrain,
            Intake intake,
            Manipulator manipulator,
            Elevator elevator,
            Elbow elbow) {

        return Commands.sequence(
                Commands.select(
                        Map.of(
                                GamePiece.CONE,
                                Commands.sequence(
                                        Commands.waitUntil(manipulator::isPoleDetected)
                                                .deadlineWith(
                                                        Commands.parallel(
                                                                driveDeltaCommand(0.6, drivetrain),
                                                                drivetrain.setCoastCommand())),
                                        Commands.parallel(
                                                manipulator
                                                        .setPincersReleasedCommand(gamePieceSupplier),
                                                elbow.setDesiredPositionCommand(ElbowPosition.LOW,
                                                        elevator),
                                                elevator.setDesiredPositionDeltaCommand(-0.2, intake,
                                                        elbow),
                                                driveDeltaCommand(-0.08, drivetrain)),
                                        Commands.either(
                                                Commands.parallel(
                                                        elbow.setDesiredPositionCommand(
                                                                ElbowPosition.HIGH, elevator),
                                                        driveDeltaCommand(-0.4, drivetrain)),
                                                Commands.none(),
                                                () -> levelSupplier.get() == Level.HIGH
                                                        && gamePieceSupplier
                                                                .get() == GamePiece.CONE)),
                                GamePiece.CUBE,
                                drivetrain.moveDeltaPathFollowingCommand(
                                        new Transform2d(
                                                new Translation2d(0.4, 0),
                                                new Rotation2d()),
                                        new PathConstraints(1,
                                                1))
                                        .andThen(Commands.sequence(
                                                manipulator.setPincersReleasedCommand(gamePieceSupplier),
                                                driveDeltaCommand(-0.18, drivetrain)))),
                        gamePieceSupplier::get));
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
    private static CommandBase scoreGamePieceImpl(
            boolean auto,
            boolean driveBackwards,
            BooleanSupplier secondaryButton,
            BooleanConsumer secondaryButtonLED,
            Supplier<GamePiece> gamePieceSupplier,
            Supplier<Level> levelSupplier,
            Drivetrain drivetrain,
            Intake intake,
            Manipulator manipulator,
            Elevator elevator,
            Elbow elbow) {
        return Commands.sequence(
                raiseElevatorCommand(gamePieceSupplier, levelSupplier, intake, manipulator, elevator,
                        elbow),
                Commands.either(
                        placePieceAutoCommand(gamePieceSupplier, levelSupplier, drivetrain, intake,
                                manipulator,
                                elevator, elbow),
                        placePieceCommand(
                                driveBackwards,
                                secondaryButton,
                                secondaryButtonLED,
                                gamePieceSupplier,
                                levelSupplier,
                                drivetrain, intake, manipulator, elevator, elbow),
                        () -> auto));
    }

    public static CommandBase scoreGamePieceCommand(
            boolean driveBackwards,
            BooleanSupplier secondaryButton,
            BooleanConsumer secondaryButtonLED,
            Drivetrain drivetrain,
            GridInterface gridInterface,
            Intake intake,
            Manipulator manipulator,
            Elevator elevator,
            Elbow elbow) {
        return Commands.either(
                scoreGamePieceImpl(
                        false,
                        driveBackwards,
                        secondaryButton,
                        secondaryButtonLED,
                        () -> gridInterface.getSetLocation().orElseThrow().gamePiece,
                        () -> gridInterface.getSetLocation().orElseThrow().level,
                        drivetrain, intake, manipulator, elevator, elbow),
                RobotStates.singularErrorCommand(() -> "Grid interface location not present"),
                () -> gridInterface.getSetLocation().isPresent()).withName("Score Game Piece Teleop");
    }

    public static CommandBase scoreGamePieceAutoCommand(
            Supplier<GamePiece> gamePieceSupplier,
            Supplier<Level> levelSupplier,
            Drivetrain drivetrain,
            Intake intake,
            Manipulator manipulator,
            Elevator elevator,
            Elbow elbow) {
        return scoreGamePieceImpl(true, true, () -> false, state -> {
        }, gamePieceSupplier, levelSupplier, drivetrain, intake, manipulator, elevator, elbow);
    }
}
