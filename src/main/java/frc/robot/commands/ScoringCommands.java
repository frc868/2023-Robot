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
import frc.robot.GridInterface;
import frc.robot.GamePieceLocation.GamePiece;
import frc.robot.GamePieceLocation.Level;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Elbow.ElbowPosition;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Manipulator;

public class ScoringCommands {
    public static CommandBase raiseElevatorCommand(
            Supplier<GamePiece> gamePieceSupplier,
            Supplier<Level> levelSupplier,
            Intake intake,
            Manipulator manipulator,
            Elevator elevator,
            Elbow elbow) {
        return Commands.parallel(
                elevator.setScoringPositionCommand(
                        gamePieceSupplier, levelSupplier, intake, elbow),
                Commands.sequence(
                        Commands.waitSeconds(0.5),
                        Commands.parallel(
                                elbow.setDesiredPositionCommand(ElbowPosition.HIGH, elevator), Commands.select(
                                        Map.of(
                                                GamePiece.CONE, manipulator.setWristUpCommand(elevator),
                                                GamePiece.CUBE, Commands.none()),
                                        gamePieceSupplier::get))));
    }

    public static CommandBase placePieceCommand(
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
                                                RobotStates.flashButtonCommand(secondaryButtonLED)),
                                        Commands.parallel(
                                                manipulator.setPincersReleasedCommand(gamePieceSupplier),
                                                elbow.setDesiredPositionCommand(ElbowPosition.LOW, elevator),
                                                elevator.setDesiredPositionDeltaCommand(-0.15, intake, elbow),
                                                RobotStates.driveDeltaCommand(-0.012, drivetrain,
                                                        new PathConstraints(4, 9)).asProxy()),
                                        manipulator.setWristDownCommand(),
                                        manipulator.setPincersOpenCommand(),
                                        elbow.setDesiredPositionCommand(ElbowPosition.MID_CONE_HIGH, elevator)),
                                GamePiece.CUBE,
                                Commands.sequence(
                                        Commands.deadline(
                                                Commands.waitUntil(secondaryButton::getAsBoolean),
                                                RobotStates.flashButtonCommand(secondaryButtonLED)),
                                        manipulator.setPincersReleasedCommand(gamePieceSupplier))),
                        gamePieceSupplier::get));
    }

    public static CommandBase placePieceAutoCommand(
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
                                        elbow.setDesiredPositionCommand(ElbowPosition.LOW, elevator),
                                        elevator.setDesiredPositionDeltaCommand(-0.15, intake, elbow),
                                        RobotStates.driveDeltaCommand(-0.012, drivetrain,
                                                new PathConstraints(4, 5))),
                                manipulator.setWristDownCommand(),
                                manipulator.setPincersOpenCommand(),
                                elbow.setDesiredPositionCommand(ElbowPosition.MID_CONE_HIGH, elevator)),
                        GamePiece.CUBE,
                        Commands.sequence(
                                drivetrain.moveDeltaPathFollowingCommand(
                                        new Transform2d(
                                                new Translation2d(0.4, 0),
                                                new Rotation2d()),
                                        new PathConstraints(4, 3)),
                                manipulator.setPincersReleasedCommand(gamePieceSupplier))),
                gamePieceSupplier::get);
    }

    public static CommandBase scorePieceCommand(
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
                RobotStates.singularErrorCommand(() -> "Grid interface location not present"),
                () -> gridInterface.getSetLocation().isPresent())
                .withName("Score Piece Teleop");
    }

    public static CommandBase scorePieceAutoCommand(
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
                        intake, manipulator, elevator, elbow).withTimeout(1.2),
                placePieceAutoCommand(
                        () -> gamePieceSupplier.get(),
                        () -> levelSupplier.get(),
                        drivetrain, intake, manipulator, elevator, elbow))
                .withName("Score Piece Auto");
    }

}
