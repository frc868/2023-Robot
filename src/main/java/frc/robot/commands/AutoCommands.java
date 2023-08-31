package frc.robot.commands;

import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.GamePieceLocation;
import frc.robot.GamePieceLocation.GamePiece;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Manipulator;

public class AutoCommands {
    public static CommandBase driveOut(
            PathPlannerTrajectory driveToGamePiece,
            Drivetrain drivetrain, Intake intake, Manipulator manipulator, Elevator elevator, Elbow elbow) {
        return Commands.deadline(
                drivetrain.pathFollowingCommand(driveToGamePiece),
                Commands.sequence(
                        Commands.waitSeconds(0.5),
                        ScoringCommands
                                .stowElevatorCommand(intake, manipulator, elevator,
                                        elbow)
                                .withTimeout(0.75)));

    }

    public static CommandBase driveAndIntake(
            PathPlannerTrajectory driveToGamePiece,
            Supplier<GamePiece> gamePieceSupplier,
            Drivetrain drivetrain, Intake intake, Manipulator manipulator, Elevator elevator, Elbow elbow) {
        return Commands.sequence(
                Commands.deadline(
                        drivetrain.pathFollowingCommand(driveToGamePiece),
                        Commands.sequence(
                                Commands.waitSeconds(0.5),
                                ScoringCommands
                                        .stowElevatorCommand(intake, manipulator, elevator,
                                                elbow)
                                        .withTimeout(0.75),
                                IntakingCommands.startAndRunIntakingCommand(gamePieceSupplier,
                                        intake,
                                        manipulator, elevator,
                                        elbow))),
                IntakingCommands.finishIntakingCommand(gamePieceSupplier, intake, manipulator,
                        elevator,
                        elbow));

    }

    public static CommandBase driveAndScore(double waitTimeForElevator,
            PathPlannerTrajectory driveToScoringLocation,
            Supplier<GamePieceLocation> gamePieceLocationSupplier,
            Drivetrain drivetrain, Intake intake, Manipulator manipulator, Elevator elevator, Elbow elbow) {
        return Commands.sequence(
                Commands.either(
                        Commands.deadline(
                                Commands.waitUntil(manipulator::isPoleDetected),
                                Commands.waitSeconds(waitTimeForElevator).andThen(
                                        ScoringCommands.raiseElevatorCommand(
                                                () -> gamePieceLocationSupplier.get().gamePiece,
                                                () -> gamePieceLocationSupplier.get().level,
                                                intake, manipulator, elevator, elbow).withTimeout(1.2)),
                                drivetrain.pathFollowingCommand(driveToScoringLocation)
                                        .andThen(drivetrain::stop)),
                        Commands.deadline(
                                drivetrain.pathFollowingCommand(driveToScoringLocation)
                                        .andThen(drivetrain::stop),
                                Commands.waitSeconds(waitTimeForElevator).andThen(
                                        ScoringCommands.raiseElevatorCommand(
                                                () -> gamePieceLocationSupplier.get().gamePiece,
                                                () -> gamePieceLocationSupplier.get().level,
                                                intake, manipulator, elevator, elbow).withTimeout(1.2))),
                        () -> gamePieceLocationSupplier.get().gamePiece == GamePiece.CONE),
                ScoringCommands.placePieceAutoCommand(
                        () -> gamePieceLocationSupplier.get().gamePiece,
                        () -> gamePieceLocationSupplier.get().level,
                        drivetrain, intake, manipulator, elevator, elbow));
    }

    public static CommandBase scorePreload(Supplier<GamePieceLocation> gamePieceLocationSupplier,
            Drivetrain drivetrain, Intake intake, Manipulator manipulator, Elevator elevator, Elbow elbow) {
        return Commands.sequence(
                ScoringCommands.raiseElevatorCommand(
                        () -> gamePieceLocationSupplier.get().gamePiece,
                        () -> gamePieceLocationSupplier.get().level,
                        intake, manipulator, elevator, elbow).withTimeout(1.2),
                Commands.either(
                        Commands.sequence(
                                Commands.deadline(
                                        Commands.waitUntil(manipulator::isPoleDetected),
                                        drivetrain.driveDeltaCommand(0.4)
                                                .finallyDo((e) -> drivetrain.stop()))),
                        Commands.sequence(
                                drivetrain.driveDeltaCommand(-0.3)
                                        .finallyDo((e) -> drivetrain.stop()),
                                ScoringCommands.placePieceAutoCommand(
                                        () -> gamePieceLocationSupplier.get().gamePiece,
                                        () -> gamePieceLocationSupplier.get().level,
                                        drivetrain, intake, manipulator, elevator, elbow)),
                        () -> gamePieceLocationSupplier.get().gamePiece == GamePiece.CONE),
                ScoringCommands.placePieceAutoCommand(
                        () -> gamePieceLocationSupplier.get().gamePiece,
                        () -> gamePieceLocationSupplier.get().level,
                        drivetrain, intake, manipulator, elevator, elbow));
    }

    public static CommandBase launchCube(Intake intake) {
        return Commands.sequence(
                intake.setCubapultReleased(),
                Commands.waitSeconds(0.1),
                intake.setRamrodsRetractedCommand());
    }
}
