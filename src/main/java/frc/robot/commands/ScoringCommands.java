package frc.robot.commands;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

// import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.GridInterface;
import frc.robot.Modes;
import frc.robot.Utils;
import frc.robot.Constants.Elbow.ElbowPosition;
import frc.robot.Constants.Elevator.ElevatorPosition;
import frc.robot.GamePieceLocation.GamePiece;
import frc.robot.GamePieceLocation.Level;
import frc.robot.Modes.RobotState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Manipulator;

public class ScoringCommands {
    public static Command raiseElevatorCommand(
            Supplier<GamePiece> gamePieceSupplier,
            Supplier<Level> levelSupplier,
            Intake intake,
            Manipulator manipulator,
            Elevator elevator,
            Elbow elbow) {
        return Commands.parallel(
                elevator.moveToScoringPositionCommand(
                        gamePieceSupplier, levelSupplier, intake, elbow).asProxy(),
                Commands.sequence(
                        Commands.waitSeconds(0.7),
                        Commands.parallel(
                                elbow.moveToPositionCommand(() -> ElbowPosition.HIGH).asProxy(),
                                Commands.select(
                                        Map.of(
                                                GamePiece.CONE, manipulator.setWristUpCommand(),
                                                GamePiece.CUBE, Commands.none()),
                                        gamePieceSupplier::get))))
                .withName("Raise Elevator");
    }

    public static Command placePieceCommand(
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
                                Commands.sequence(
                                        Commands.deadline(
                                                Commands.waitUntil(
                                                        () -> (manipulator.isPoleDetected()
                                                                || secondaryButton.getAsBoolean())),
                                                Utils.flashButtonCommand(secondaryButtonLED)),
                                        Commands.parallel(
                                                manipulator.setPincersReleasedCommand(gamePieceSupplier),
                                                elbow.moveToPositionCommand(() -> ElbowPosition.LOW).asProxy(),
                                                elevator.movePositionDeltaCommand(() -> -0.15).asProxy())
                                                .withTimeout(0.5),
                                        // drivetrain.driveDistanceDeltaCommand(-0.012,
                                        // new PathConstraints(4, 9))),)))
                                        manipulator.setWristDownCommand(),
                                        manipulator.setPincersOpenCommand(),
                                        elbow.moveToPositionCommand(() -> ElbowPosition.MID_CONE_HIGH).asProxy()),
                                GamePiece.CUBE,
                                Commands.sequence(
                                        Commands.deadline(
                                                Commands.waitUntil(secondaryButton::getAsBoolean),
                                                Utils.flashButtonCommand(secondaryButtonLED)),
                                        manipulator.setPincersReleasedCommand(gamePieceSupplier))),
                        gamePieceSupplier::get))
                .withName("Place Piece");
    }

    public static Command placePieceAutoCommand(
            Supplier<GamePiece> gamePieceSupplier,
            Supplier<Level> levelSupplier,
            Drivetrain drivetrain,
            Intake intake,
            Manipulator manipulator,
            Elevator elevator,
            Elbow elbow) {
        return Commands.select(
                Map.of(
                        GamePiece.CONE,
                        Commands.sequence(
                                Commands.parallel(
                                        manipulator.setPincersReleasedCommand(gamePieceSupplier),
                                        elbow.moveToPositionCommand(() -> ElbowPosition.LOW).asProxy(),
                                        elevator.movePositionDeltaCommand(() -> -0.15).asProxy()),
                                // drivetrain.driveDistanceDeltaCommand(-0.012, new PathConstraints(4, 5))),
                                manipulator.setWristDownCommand(),
                                manipulator.setPincersOpenCommand(),
                                elbow.moveToPositionCommand(() -> ElbowPosition.MID_CONE_HIGH).asProxy()),
                        GamePiece.CUBE,
                        Commands.sequence(
                                // drivetrain.driveDistanceDeltaCommand(0.4, new PathConstraints(4, 3)),
                                manipulator.setPincersReleasedCommand(gamePieceSupplier))),
                gamePieceSupplier::get)
                .withName("Place Piece Auto");
    }

    public static Command fullScoreSequenceCommand(
            BooleanSupplier secondaryButton,
            BooleanConsumer secondaryButtonLED,
            Drivetrain drivetrain,
            GridInterface gridInterface,
            Intake intake,
            Manipulator manipulator,
            Elevator elevator,
            Elbow elbow) {
        return Commands.either(
                Commands.sequence(
                        raiseElevatorCommand(
                                () -> gridInterface.getSetLocation().orElseThrow().gamePiece,
                                () -> gridInterface.getSetLocation().orElseThrow().level,
                                intake, manipulator, elevator, elbow).withTimeout(1.2),
                        placePieceCommand(
                                secondaryButton,
                                secondaryButtonLED,
                                () -> gridInterface.getSetLocation().orElseThrow().gamePiece,
                                () -> gridInterface.getSetLocation().orElseThrow().level,
                                drivetrain, intake, manipulator, elevator, elbow)),
                Modes.singularErrorCommand(() -> "Grid interface location not present"),
                () -> gridInterface.getSetLocation().isPresent())
                .withName("Full Score Sequence");
    }

    public static Command fullScoreSequenceCommand(
            BooleanSupplier secondaryButton,
            Supplier<GamePiece> gamePiece,
            Supplier<Level> level,
            Drivetrain drivetrain,
            Intake intake,
            Manipulator manipulator,
            Elevator elevator,
            Elbow elbow) {
        return Commands.sequence(
                raiseElevatorCommand(
                        gamePiece::get, level::get,
                        intake, manipulator, elevator, elbow).withTimeout(1.2),
                placePieceCommand(
                        secondaryButton,
                        (b) -> {
                        },
                        gamePiece::get, level::get,
                        drivetrain, intake, manipulator, elevator, elbow));
    }

    public static Command fullScoreSequenceAutoCommand(
            Supplier<GamePiece> gamePieceSupplier,
            Supplier<Level> levelSupplier,
            Drivetrain drivetrain,
            Intake intake,
            Manipulator manipulator,
            Elevator elevator,
            Elbow elbow) {
        return Commands.sequence(
                raiseElevatorCommand(
                        () -> gamePieceSupplier.get(),
                        () -> levelSupplier.get(),
                        intake, manipulator, elevator, elbow),
                placePieceAutoCommand(
                        () -> gamePieceSupplier.get(),
                        () -> levelSupplier.get(),
                        drivetrain, intake, manipulator, elevator, elbow))
                .withName("Full Score Sequence Auto");
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
    public static Command stowElevatorCommand(
            Intake intake,
            Manipulator manipulator,
            Elevator elevator,
            Elbow elbow) {
        return Commands.sequence(
                manipulator.setWristDownCommand(),
                manipulator.setPincersClosedCommand(),
                elevator.moveToPositionCommand(() -> ElevatorPosition.BOTTOM).asProxy(),
                Modes.setRobotStateCommand(() -> RobotState.SEEKING))
                .withName("Stow Elevator");
    }

}
