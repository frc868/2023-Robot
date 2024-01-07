package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;
import com.techhounds.houndutil.houndauto.AutoRoutine;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.GamePieceLocation;
import frc.robot.GamePieceLocation.GamePiece;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Manipulator;

public class Autos {
    public static AutoRoutine northThreePiece(Drivetrain drivetrain, Intake intake,
            Manipulator manipulator, Elevator elevator, Elbow elbow) {

        PathPlannerPath path1 = PathPlannerPath.fromChoreoTrajectory("northThreePiece1");
        PathPlannerPath path2 = PathPlannerPath.fromChoreoTrajectory("northThreePiece2");

        Command command = Commands.sequence(
                manipulator.setWristDownCommand(),
                manipulator.setPincersClosedCommand(),
                ScoringCommands.fullScoreSequenceAutoCommand(
                        () -> GamePieceLocation.H1.gamePiece,
                        () -> GamePieceLocation.H1.level,
                        drivetrain, intake, manipulator,
                        elevator, elbow),
                Commands.parallel(
                        Commands.waitSeconds(0.75).andThen(drivetrain.followPathCommand(path1)),
                        Commands.sequence(
                                ScoringCommands.stowElevatorCommand(intake, manipulator, elevator, elbow),
                                IntakingCommands.startAndRunIntakingCommand(
                                        () -> GamePiece.CUBE, intake, manipulator, elevator, elbow).withTimeout(2),
                                IntakingCommands.finishIntakingCommand(
                                        () -> GamePiece.CUBE, intake, manipulator, elevator, elbow),
                                IntakingCommands.startAndRunEjectingCommand(
                                        () -> GamePiece.CUBE, intake, manipulator, elevator, elbow).withTimeout(1),
                                IntakingCommands.finishEjectingCommand(
                                        () -> GamePiece.CUBE, intake, manipulator, elevator, elbow))),
                Commands.parallel(
                        drivetrain.followPathCommand(path2),
                        Commands.sequence(
                                Commands.waitSeconds(1),
                                IntakingCommands.startAndRunIntakingCommand(
                                        () -> GamePiece.CUBE, intake, manipulator, elevator, elbow).withTimeout(2),
                                IntakingCommands.finishIntakingCommand(
                                        () -> GamePiece.CUBE, intake, manipulator, elevator, elbow),
                                IntakingCommands.startAndRunEjectingCommand(
                                        () -> GamePiece.CUBE, intake, manipulator, elevator, elbow).withTimeout(1),
                                IntakingCommands.finishEjectingCommand(
                                        () -> GamePiece.CUBE, intake, manipulator, elevator, elbow))));

        return new AutoRoutine("North: 3 Piece",
                command, List.of(path1, path2),
                path1.getPreviewStartingHolonomicPose()
                        .plus(new Transform2d(new Translation2d(), new Rotation2d(Math.PI))));
    }
}
