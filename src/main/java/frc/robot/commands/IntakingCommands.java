package frc.robot.commands;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.GamePieceLocation.GamePiece;
import frc.robot.commands.RobotStates.RobotState;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Elbow.ElbowPosition;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorPosition;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Manipulator;

public class IntakingCommands {
    public static CommandBase runIntakingCommand(
            Supplier<GamePiece> gamePieceSupplier,
            Intake intake,
            Manipulator manipulator,
            Elevator elevator,
            Elbow elbow) {
        return Commands.sequence(
                Commands.parallel(
                        elevator.setDesiredPositionCommand(ElevatorPosition.BOTTOM, intake,
                                elbow).withTimeout(0.75),
                        Commands.select(
                                Map.of(
                                        GamePiece.CONE,
                                        elbow.setDesiredPositionCommand(ElbowPosition.CONE_PICKUP, elevator),
                                        GamePiece.CUBE,
                                        elbow.setDesiredPositionCommand(ElbowPosition.MID, elevator)),
                                () -> gamePieceSupplier.get())),
                manipulator.setPincersReleasedCommand(() -> gamePieceSupplier.get()),
                Commands.runOnce(() -> RobotStates.setIntaking(true)),
                Commands.parallel(
                        Commands.select(
                                Map.of(
                                        GamePiece.CONE, Commands.none(),
                                        GamePiece.CUBE, intake.setPassoversExtendedCommand(elevator)
                                                .andThen(intake.runPassoverMotorsCommand())),
                                () -> gamePieceSupplier.get()),
                        manipulator.setPincersReleasedCommand(() -> gamePieceSupplier.get())).repeatedly())
                .finallyDo((d) -> RobotStates.setIntaking(false));
    }

    public static CommandBase endIntakingCommand(
            Supplier<GamePiece> gamePieceSupplier,
            Intake intake,
            Manipulator manipulator,
            Elevator elevator,
            Elbow elbow) {
        return Commands.sequence(
                Commands.runOnce(() -> RobotStates.setIntaking(false)),
                manipulator.setPincersPincingCommand(() -> gamePieceSupplier.get()),
                intake.setPassoversRetractedCommand(elevator),
                elevator.setDesiredPositionDeltaCommand(0.02, intake, elbow),
                Commands.waitSeconds(0.2)
                        .andThen(elbow.setDesiredPositionCommand(ElbowPosition.HIGH, elevator)),
                RobotStates.setCurrentStateCommand(RobotState.SCORING),
                RobotStates.clearIntakeModeCommand());
    }

    public static CommandBase intakePieceCommand(
            BooleanSupplier secondaryButton,
            Intake intake,
            Manipulator manipulator,
            Elevator elevator,
            Elbow elbow) {
        return Commands.either(
                Commands.sequence(
                        Commands.deadline(
                                Commands.waitUntil(secondaryButton),
                                runIntakingCommand(() -> RobotStates.getIntakeMode().orElseThrow(),
                                        intake, manipulator, elevator, elbow)),
                        endIntakingCommand(() -> RobotStates.getIntakeMode().orElseThrow(),
                                intake, manipulator, elevator, elbow)),
                RobotStates.singularErrorCommand(() -> "Intake mode not present"),
                () -> RobotStates.getIntakeMode().isPresent()).withName("Intake Game Piece");
    }

    public static CommandBase runEjectingPieceCommand(
            Supplier<GamePiece> gamePieceSupplier,
            Intake intake,
            Manipulator manipulator,
            Elevator elevator,
            Elbow elbow) {
        return Commands.either(
                Commands.sequence(
                        RobotStates.setIntakeModeCommand(() -> gamePieceSupplier.get()),
                        Commands.parallel(
                                elevator.setDesiredPositionCommand(ElevatorPosition.BOTTOM, intake, elbow)
                                        .withTimeout(0.75),
                                Commands.select(
                                        Map.of(
                                                GamePiece.CONE,
                                                elbow.setDesiredPositionCommand(ElbowPosition.CONE_PICKUP, elevator),
                                                GamePiece.CUBE,
                                                elbow.setDesiredPositionCommand(ElbowPosition.MID, elevator)),
                                        () -> gamePieceSupplier.get())),
                        manipulator.setPincersReleasedCommand(() -> gamePieceSupplier.get()),
                        Commands.parallel(
                                Commands.select(
                                        Map.of(
                                                GamePiece.CONE, Commands.none(),
                                                GamePiece.CUBE, intake.setPassoversExtendedCommand(elevator)
                                                        .andThen(intake.reversePassoverMotorsCommand())),
                                        () -> gamePieceSupplier.get()),
                                manipulator.setPincersReleasedCommand(() -> gamePieceSupplier.get())
                                        .repeatedly())),
                RobotStates.singularErrorCommand(() -> "Intake mode not present"),
                () -> RobotStates.getIntakeMode().isPresent()).withName("Intake Game Piece");

    }

    public static CommandBase endEjectingPieceCommand(
            Supplier<GamePiece> gamePieceSupplier,
            Intake intake,
            Manipulator manipulator,
            Elevator elevator,
            Elbow elbow) {
        return Commands.sequence(
                manipulator.setPincersClosedCommand(),
                intake.setPassoversRetractedCommand(elevator),
                Commands.waitSeconds(0.2)
                        .andThen(elbow.setDesiredPositionCommand(ElbowPosition.HIGH, elevator)),
                RobotStates.setCurrentStateCommand(RobotState.SCORING),
                RobotStates.clearIntakeModeCommand());
    }

    public static CommandBase ejectPieceCommand(
            BooleanSupplier secondaryButton,
            Intake intake,
            Manipulator manipulator,
            Elevator elevator,
            Elbow elbow) {
        return Commands.either(
                Commands.sequence(
                        Commands.waitUntil(secondaryButton).deadlineWith(
                                runEjectingPieceCommand(() -> RobotStates.getIntakeMode().orElseThrow(),
                                        intake, manipulator, elevator, elbow)),
                        endEjectingPieceCommand(() -> RobotStates.getIntakeMode().orElseThrow(),
                                intake, manipulator, elevator, elbow)),
                RobotStates.singularErrorCommand(() -> "Intake mode not present"),
                () -> RobotStates.getIntakeMode().isPresent()).withName("Intake Game Piece");

    }
}
