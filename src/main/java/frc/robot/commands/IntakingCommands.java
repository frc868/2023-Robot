package frc.robot.commands;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Modes;
import frc.robot.Utils;
import frc.robot.GamePieceLocation.GamePiece;
import frc.robot.Modes.RobotState;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Elbow.ElbowPosition;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorPosition;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Manipulator;

public class IntakingCommands {
    public static CommandBase startAndRunIntakingCommand(
            Supplier<GamePiece> gamePieceSupplier,
            Intake intake,
            Manipulator manipulator,
            Elevator elevator,
            Elbow elbow) {
        return Commands.sequence(
                manipulator.setWristDownCommand(),
                Commands.parallel(
                        elevator.moveToPositionCommand(() -> ElevatorPosition.BOTTOM).withTimeout(0.75).asProxy(),
                        Commands.select(
                                Map.of(
                                        GamePiece.CONE,
                                        elbow.moveToPositionCommand(() -> ElbowPosition.CONE_PICKUP).asProxy(),
                                        GamePiece.CUBE,
                                        elbow.moveToPositionCommand(() -> ElbowPosition.MID).asProxy()),
                                () -> gamePieceSupplier.get())),
                manipulator.setPincersReleasedCommand(() -> gamePieceSupplier.get()),
                Commands.runOnce(() -> Modes.setIntaking(true)),
                Commands.parallel(
                        Commands.select(
                                Map.of(
                                        GamePiece.CONE, Commands.none(),
                                        GamePiece.CUBE, intake.setPassoversExtendedCommand(elevator)
                                                .andThen(intake.runPassoverMotorsCommand())),
                                () -> gamePieceSupplier.get()),
                        manipulator.setPincersReleasedCommand(() -> gamePieceSupplier.get())).repeatedly())
                .finallyDo((d) -> Modes.setIntaking(false))
                .withName("Start and Run Intaking");
    }

    public static CommandBase finishIntakingCommand(
            Supplier<GamePiece> gamePieceSupplier,
            Intake intake,
            Manipulator manipulator,
            Elevator elevator,
            Elbow elbow) {
        return Commands.sequence(
                Commands.runOnce(() -> Modes.setIntaking(false)),
                manipulator.setPincersPincingCommand(() -> gamePieceSupplier.get()),
                intake.setPassoversRetractedCommand(elevator),
                elevator.movePositionDeltaCommand(() -> 0.02).withTimeout(0.1).asProxy(),
                Commands.waitSeconds(0.2)
                        .andThen(elbow.moveToPositionCommand(() -> ElbowPosition.HIGH).asProxy()),
                Modes.setRobotStateCommand(() -> RobotState.SCORING),
                Modes.clearIntakeModeCommand()).withName("Finish Intaking");
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
                                startAndRunIntakingCommand(() -> Modes.getIntakeMode().orElseThrow(),
                                        intake, manipulator, elevator, elbow)),
                        finishIntakingCommand(() -> Modes.getIntakeMode().orElseThrow(),
                                intake, manipulator, elevator, elbow)),
                Modes.singularErrorCommand(() -> "Intake mode not present"),
                () -> Modes.getIntakeMode().isPresent())
                .withName("Intake Game Piece");
    }

    public static CommandBase startAndRunEjectingCommand(
            Supplier<GamePiece> gamePieceSupplier,
            Intake intake,
            Manipulator manipulator,
            Elevator elevator,
            Elbow elbow) {
        return Commands.either(
                Commands.sequence(
                        Modes.setIntakeModeCommand(() -> gamePieceSupplier.get()),
                        Commands.parallel(
                                elevator.moveToPositionCommand(() -> ElevatorPosition.BOTTOM)
                                        .withTimeout(0.75).asProxy(),
                                Commands.select(
                                        Map.of(
                                                GamePiece.CONE,
                                                elbow.moveToPositionCommand(() -> ElbowPosition.CONE_PICKUP).asProxy(),
                                                GamePiece.CUBE,
                                                elbow.moveToPositionCommand(() -> ElbowPosition.MID).asProxy()),
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
                Modes.singularErrorCommand(() -> "Intake mode not present"),
                () -> Modes.getIntakeMode().isPresent()).withName("Start and Run Ejecting");

    }

    public static CommandBase finishEjectingCommand(
            Supplier<GamePiece> gamePieceSupplier,
            Intake intake,
            Manipulator manipulator,
            Elevator elevator,
            Elbow elbow) {
        return Commands.sequence(
                manipulator.setPincersClosedCommand(),
                intake.setPassoversRetractedCommand(elevator),
                Commands.waitSeconds(0.2)
                        .andThen(elbow.moveToPositionCommand(() -> ElbowPosition.HIGH).asProxy()),
                Modes.setRobotStateCommand(() -> RobotState.SCORING),
                Modes.clearIntakeModeCommand()).withName("Finish Ejecting");
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
                                startAndRunEjectingCommand(() -> Modes.getIntakeMode().orElseThrow(),
                                        intake, manipulator, elevator, elbow)),
                        finishEjectingCommand(() -> Modes.getIntakeMode().orElseThrow(),
                                intake, manipulator, elevator, elbow)),
                Modes.singularErrorCommand(() -> "Intake mode not present"),
                () -> Modes.getIntakeMode().isPresent()).withName("Eject Game Piece");

    }

    public static CommandBase humanPlayerPickupCommand(
            BooleanSupplier secondaryButton,
            BooleanConsumer secondaryButtonLED,
            Supplier<GamePiece> gamePieceSupplier,
            Intake intake,
            Manipulator manipulator,
            Elevator elevator,
            Elbow elbow) {
        return Commands.sequence(
                Modes.setIntakeModeCommand(gamePieceSupplier),
                Commands.parallel(
                        elevator.moveToPositionCommand(() -> ElevatorPosition.SINGLE_SUBSTATION_PICKUP).asProxy(),
                        elbow.moveToPositionCommand(() -> ElbowPosition.SINGLE_SUBSTATION_PICKUP).asProxy()),
                Commands.waitSeconds(0.5),
                manipulator.setWristUpCommand(),
                manipulator.setPincersReleasedCommand(gamePieceSupplier),
                Commands.deadline(
                        Commands.waitUntil(secondaryButton::getAsBoolean),
                        Utils.flashButtonCommand(secondaryButtonLED)),
                manipulator.setPincersPincingCommand(gamePieceSupplier),
                Modes.clearIntakeModeCommand())
                .withName("Human Player Pickup");
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
    public static CommandBase humanPlayerStowElevatorCommand(
            Intake intake,
            Manipulator manipulator,
            Elevator elevator,
            Elbow elbow) {
        return Commands.sequence(
                elevator.moveToPositionCommand(() -> ElevatorPosition.BOTTOM).asProxy(),
                manipulator.setWristDownCommand(),
                elbow.moveToPositionCommand(() -> ElbowPosition.HIGH).asProxy(),
                Modes.setRobotStateCommand(() -> RobotState.SCORING))
                .withName("Human Player Stow Elevator");
    }

}
