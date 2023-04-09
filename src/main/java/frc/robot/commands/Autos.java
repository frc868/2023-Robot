package frc.robot.commands;

import java.util.function.Supplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.techhounds.houndutil.houndauto.AutoManager;
import com.techhounds.houndutil.houndauto.AutoPath;
import com.techhounds.houndutil.houndauto.AutoTrajectoryCommand;
import com.techhounds.houndutil.houndauto.trajectoryloader.TrajectoryLoader;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.GamePieceLocation;
import frc.robot.GamePieceLocation.GamePiece;
import frc.robot.commands.RobotStates.RobotState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Manipulator;

public class Autos {
    public static SwerveAutoBuilder getBuilder(Drivetrain drivetrain) {
        return new SwerveAutoBuilder(
                drivetrain::getPose,
                (p) -> drivetrain.resetPoseEstimator(p),
                Constants.Geometries.Drivetrain.KINEMATICS,
                new PIDConstants(Constants.Gains.Trajectories.xkP, 0, 0),
                new PIDConstants(Constants.Gains.Trajectories.thetakP, 0, 0),
                (s) -> drivetrain.setModuleStates(s, true, true),
                AutoManager.getInstance().getEventMap(),
                false,
                drivetrain);
    }

    public static Supplier<AutoTrajectoryCommand> pathPlannerTrajectory(AutoPath autoPath, Drivetrain drivetrain) {
        return () -> new AutoTrajectoryCommand(autoPath, getBuilder(drivetrain).fullAuto(autoPath.getTrajectories()));
    }

    /**
     * Creates the DoNothing command.
     * 
     * @param autoPath   the {@link AutoPath} containing the Figure 8 trajectory.
     * @param drivetrain the drivetrain
     * @return the command
     */
    public static Supplier<AutoTrajectoryCommand> doNothing() {
        return () -> new AutoTrajectoryCommand(new Pose2d());
    }

    public static CommandBase driveIntakeScoreGamePiece(
            boolean raiseElevator,
            PathPlannerTrajectory driveToGamePiece, PathPlannerTrajectory driveToScoringLocation,
            double waitTimeForElevator,
            GamePieceLocation gamePieceLocation,
            Drivetrain drivetrain, Intake intake, Manipulator manipulator, Elevator elevator, Elbow elbow) {
        return Commands.sequence(
                Commands.deadline(
                        drivetrain.pathFollowingCommand(driveToGamePiece),
                        Commands.sequence(
                                Commands.waitSeconds(0.5),
                                RobotStates
                                        .stowElevatorCommand(intake, manipulator, elevator,
                                                elbow)
                                        .withTimeout(0.75),
                                IntakingCommands.runIntakingCommand(() -> gamePieceLocation.gamePiece,
                                        intake,
                                        manipulator, elevator,
                                        elbow))),
                IntakingCommands.endIntakingCommand(() -> gamePieceLocation.gamePiece, intake, manipulator,
                        elevator,
                        elbow),
                Commands.either(
                        Commands.sequence(
                                Commands.deadline(
                                        Commands.waitUntil(manipulator::isPoleDetected),
                                        Commands.waitSeconds(waitTimeForElevator).andThen(
                                                ScoringCommands.raiseElevatorCommand(
                                                        () -> gamePieceLocation.gamePiece,
                                                        () -> gamePieceLocation.level,
                                                        intake, manipulator, elevator, elbow).withTimeout(1.2)),
                                        drivetrain.pathFollowingCommand(driveToScoringLocation)
                                                .andThen(drivetrain::stop)),
                                ScoringCommands.placePieceAutoCommand(
                                        () -> gamePieceLocation.gamePiece,
                                        () -> gamePieceLocation.level,
                                        drivetrain, intake, manipulator, elevator, elbow)),
                        drivetrain.pathFollowingCommand(driveToScoringLocation)
                                .andThen(drivetrain::stop),
                        () -> raiseElevator));
    }

    public static CommandBase launchCube(Intake intake) {
        return Commands.sequence(
                intake.setCubapultReleased(),
                Commands.waitSeconds(0.1),
                intake.setRamrodsRetractedCommand());
    }

    // public static AutoTrajectoryCommand threePieceLinkN(Drivetrain drivetrain,
    // Intake intake,
    // Manipulator manipulator, Elevator elevator, Elbow elbow) {
    // AutoPath autoPath = TrajectoryLoader.getAutoPath("3 Piece Link N");
    // return new AutoTrajectoryCommand(
    // FieldConstants.getStartingCubapultPose(GamePieceLocation.H1),
    // autoPath,
    // Commands.sequence(
    // launchCube(intake),
    // driveIntakeScoreGamePiece(true, autoPath.getTrajectories().get(0),
    // autoPath.getTrajectories().get(1),
    // autoPath.getTrajectories().get(1).getMarkers().get(0).timeSeconds,
    // GamePieceLocation.I2, drivetrain, intake,
    // manipulator,
    // elevator, elbow),
    // driveIntakeScoreGamePiece(false, autoPath.getTrajectories().get(2),
    // autoPath.getTrajectories().get(3),
    // autoPath.getTrajectories().get(3).getMarkers().get(0).timeSeconds,
    // GamePieceLocation.G2, drivetrain, intake,
    // manipulator,
    // elevator, elbow)));
    // // Commands.parallel(
    // // RobotStates.stowElevatorCommand(intake, manipulator, elevator, elbow))));
    // }

    public static AutoTrajectoryCommand twoPieceCubeN(Drivetrain drivetrain, Intake intake,
            Manipulator manipulator, Elevator elevator, Elbow elbow) {
        AutoPath autoPath = TrajectoryLoader.getAutoPath("3 Piece Link N");
        return new AutoTrajectoryCommand(
                FieldConstants.getStartingCubapultPose(GamePieceLocation.H1),
                autoPath,
                Commands.sequence(
                        launchCube(intake),
                        Commands.deadline(
                                drivetrain.pathFollowingCommand(autoPath.getTrajectories().get(0)),
                                Commands.sequence(
                                        Commands.waitSeconds(0.5),
                                        RobotStates
                                                .stowElevatorCommand(intake, manipulator, elevator,
                                                        elbow)
                                                .withTimeout(0.75),
                                        IntakingCommands.runIntakingCommand(() -> GamePiece.CUBE,
                                                intake,
                                                manipulator, elevator,
                                                elbow))),
                        IntakingCommands.endIntakingCommand(() -> GamePiece.CUBE, intake, manipulator,
                                elevator,
                                elbow),
                        drivetrain.pathFollowingCommand(autoPath.getTrajectories().get(1)),
                        ScoringCommands.scorePieceAutoCommand(() -> GamePieceLocation.H2.gamePiece,
                                () -> GamePieceLocation.H2.level,
                                drivetrain, intake, manipulator,
                                elevator, elbow),
                        Commands.waitSeconds(1),
                        RobotStates.driveDeltaCommand(-0.4, drivetrain, new PathConstraints(4, 3))));
        // Commands.parallel(
        // RobotStates.stowElevatorCommand(intake, manipulator, elevator, elbow))));
    }

    public static AutoTrajectoryCommand onePieceMobilityN(Drivetrain drivetrain, Intake intake,
            Manipulator manipulator, Elevator elevator, Elbow elbow) {
        AutoPath autoPath = TrajectoryLoader.getAutoPath("1 Piece Mobility N");
        return new AutoTrajectoryCommand(
                FieldConstants.getStartingPoseFacingGrid(GamePieceLocation.H1),
                autoPath,
                Commands.sequence(
                        RobotStates.autoDriveCommand(new PathConstraints(3, 2), () -> RobotState.SCORING,
                                () -> GamePieceLocation.H1,
                                drivetrain),
                        ScoringCommands.scorePieceAutoCommand(() -> GamePieceLocation.E1.gamePiece,
                                () -> GamePieceLocation.H1.level,
                                drivetrain, intake, manipulator,
                                elevator, elbow),
                        Commands.waitSeconds(1),
                        RobotStates.driveDeltaCommand(-0.4, drivetrain, new PathConstraints(4, 3)),
                        RobotStates.stowElevatorCommand(intake, manipulator, elevator, elbow),
                        drivetrain.pathFollowingCommand(autoPath.getTrajectories().get(0))));
    }

    public static AutoTrajectoryCommand onePieceChargeMobilityM(Drivetrain drivetrain,
            Intake intake,
            Manipulator manipulator, Elevator elevator, Elbow elbow) {
        AutoPath autoPath = TrajectoryLoader.getAutoPath("1 Piece Charge Mobility M");
        return new AutoTrajectoryCommand(
                FieldConstants.getStartingPoseFacingGrid(GamePieceLocation.E1), autoPath,
                Commands.sequence(
                        RobotStates.autoDriveCommand(new PathConstraints(3, 2), () -> RobotState.SCORING,
                                () -> GamePieceLocation.E1,
                                drivetrain),
                        ScoringCommands.scorePieceAutoCommand(() -> GamePieceLocation.E1.gamePiece,
                                () -> GamePieceLocation.E1.level,
                                drivetrain, intake, manipulator,
                                elevator, elbow),
                        Commands.waitSeconds(1),
                        RobotStates.driveDeltaCommand(-0.4, drivetrain, new PathConstraints(4, 3)),
                        RobotStates.stowElevatorCommand(intake, manipulator, elevator, elbow),
                        drivetrain.pathFollowingCommand(
                                autoPath.getTrajectories().get(0)),
                        drivetrain.chargeStationBalanceCommand()));
    }

    public static AutoTrajectoryCommand onePieceChargeM(Drivetrain drivetrain,
            Intake intake,
            Manipulator manipulator, Elevator elevator, Elbow elbow) {
        AutoPath autoPath = TrajectoryLoader.getAutoPath("1 Piece Charge M");
        return new AutoTrajectoryCommand(
                FieldConstants.getStartingPoseFacingGrid(GamePieceLocation.E1), autoPath,
                Commands.sequence(
                        RobotStates.autoDriveCommand(new PathConstraints(3, 2), () -> RobotState.SCORING,
                                () -> GamePieceLocation.E1,
                                drivetrain),
                        ScoringCommands.scorePieceAutoCommand(() -> GamePieceLocation.E1.gamePiece,
                                () -> GamePieceLocation.E1.level,
                                drivetrain, intake, manipulator,
                                elevator, elbow),
                        Commands.waitSeconds(1),
                        RobotStates.driveDeltaCommand(-0.4, drivetrain, new PathConstraints(4, 3)),
                        RobotStates.stowElevatorCommand(intake, manipulator, elevator, elbow),
                        drivetrain.pathFollowingCommand(
                                autoPath.getTrajectories().get(0)),
                        drivetrain.chargeStationBalanceCommand()));
    }

    public static AutoTrajectoryCommand onePieceMobilityS(Drivetrain drivetrain,
            Intake intake,
            Manipulator manipulator, Elevator elevator, Elbow elbow) {
        AutoPath autoPath = TrajectoryLoader.getAutoPath("1 Piece Mobility S");
        return new AutoTrajectoryCommand(
                FieldConstants.getStartingPoseFacingGrid(GamePieceLocation.B1), autoPath,
                Commands.sequence(
                        RobotStates.autoDriveCommand(new PathConstraints(3, 2), () -> RobotState.SCORING,
                                () -> GamePieceLocation.B1,
                                drivetrain),
                        ScoringCommands.scorePieceAutoCommand(() -> GamePieceLocation.E1.gamePiece,
                                () -> GamePieceLocation.B1.level,
                                drivetrain, intake, manipulator,
                                elevator, elbow),
                        Commands.waitSeconds(1),
                        RobotStates.driveDeltaCommand(-0.4, drivetrain, new PathConstraints(4, 3)),
                        RobotStates.stowElevatorCommand(intake, manipulator, elevator, elbow),
                        drivetrain.pathFollowingCommand(
                                autoPath.getTrajectories().get(0))));
    }
}
