package frc.robot.commands;

import java.util.function.Supplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.techhounds.houndutil.houndauto.AutoManager;
import com.techhounds.houndutil.houndauto.AutoPath;
import com.techhounds.houndutil.houndauto.AutoTrajectoryCommand;
import com.techhounds.houndutil.houndauto.trajectoryloader.TrajectoryLoader;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

    public static Pose2d getStartingPose(Pose2d autoDrivePose) {
        return new Pose2d(1.823, autoDrivePose.getY(), Rotation2d.fromDegrees(180));
    }

    /**
     * Creates the circle trajectory command.
     * 
     * @param autoPath   the {@link AutoPath} containing the circle trajectory.
     * @param drivetrain the drivetrain
     * @return the command
     */
    public static Supplier<AutoTrajectoryCommand> circle(AutoPath autoPath, Drivetrain drivetrain) {
        PathPlannerTrajectory path = autoPath.getTrajectories().get(0);
        return () -> new AutoTrajectoryCommand(autoPath, new FollowPathWithEvents(
                drivetrain.pathFollowingCommand(path),
                path.getMarkers(),
                AutoManager.getInstance().getEventMap()));
    }

    /**
     * Creates the Figure 8 trajectory command.
     * 
     * @param autoPath   the {@link AutoPath} containing the Figure 8 trajectory.
     * @param drivetrain the drivetrain
     * @return the command
     */
    public static Supplier<AutoTrajectoryCommand> figure8(AutoPath autoPath, Drivetrain drivetrain) {
        return () -> new AutoTrajectoryCommand(autoPath, drivetrain.spinningPathFollowingCommand(
                autoPath.getTrajectories().get(0)));
    }

    /**
     * Creates the Figure 8 trajectory command.
     * 
     * @param autoPath   the {@link AutoPath} containing the Figure 8 trajectory.
     * @param drivetrain the drivetrain
     * @return the command
     */
    public static Supplier<AutoTrajectoryCommand> pathPlannerTrajectory(AutoPath autoPath, Drivetrain drivetrain) {
        return () -> new AutoTrajectoryCommand(autoPath, getBuilder(drivetrain).fullAuto(autoPath.getTrajectories()));
    }

    public static CommandBase fullAutoScoreWithMovement(
            GamePieceLocation gamePieceLocation,
            Drivetrain drivetrain,
            Intake intake,
            Manipulator manipulator, Elevator elevator, Elbow elbow) {
        return fullAutoScoreWithMovement(new PathConstraints(3, 2), gamePieceLocation, drivetrain, intake, manipulator,
                elevator, elbow);
    }

    public static CommandBase fullAutoScoreWithMovement(
            PathConstraints constraints,
            GamePieceLocation gamePieceLocation,
            Drivetrain drivetrain,
            Intake intake,
            Manipulator manipulator, Elevator elevator, Elbow elbow) {
        return Commands.sequence(
                RobotStates.autoDriveCommand(constraints, () -> RobotState.SCORING, () -> gamePieceLocation,
                        drivetrain),
                Scoring.scoreGamePieceAutoCommand(() -> gamePieceLocation.gamePiece, () -> gamePieceLocation.level,
                        drivetrain, intake, manipulator,
                        elevator, elbow),
                RobotStates.driveDeltaCommand(-0.5, drivetrain),
                RobotStates.stowElevatorCommand(intake, manipulator, elevator, elbow));
    }

    public static CommandBase fullAutoIntakeWithMovement(
            GamePiece gamePiece,
            Drivetrain drivetrain, Intake intake,
            Manipulator manipulator, Elevator elevator, Elbow elbow) {
        return RobotStates.setIntakeModeCommand(gamePiece).andThen(
                RobotStates.intakeGamePieceAutoCommand(intake, manipulator, elevator, elbow));
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

    /**
     * Creates the 2 Piece N trajectory command.
     * 
     * @param autoPath   the {@link AutoPath} containing the Figure 8 trajectory.
     * @param drivetrain the drivetrain
     * @return the command
     */
    public static Supplier<AutoTrajectoryCommand> onePieceM(AutoPath autoPath, Drivetrain drivetrain,
            Intake intake,
            Manipulator manipulator, Elevator elevator, Elbow elbow) {
        return () -> new AutoTrajectoryCommand(
                getStartingPose(FieldConstants.Blue.MiddleGrid.CUBE_5), autoPath,
                Commands.sequence(
                        // RobotStates.initializeMechanisms(intake, manipulator, elevator,
                        // elbow).withTimeout(2),
                        fullAutoScoreWithMovement(GamePieceLocation.E1, drivetrain, intake, manipulator, elevator,
                                elbow),
                        drivetrain.pathFollowingCommand(
                                autoPath.getTrajectories().get(0)).asProxy(),
                        drivetrain.chargeStationBalanceCommand().asProxy()));
    }

    /**
     * Creates the 2 Piece N trajectory command.
     * 
     * @param autoPath   the {@link AutoPath} containing the Figure 8 trajectory.
     * @param drivetrain the drivetrain
     * @return the command
     */
    public static Supplier<AutoTrajectoryCommand> onePieceN(Drivetrain drivetrain, Intake intake,
            Manipulator manipulator, Elevator elevator, Elbow elbow) {
        return () -> new AutoTrajectoryCommand(getStartingPose(FieldConstants.Blue.RightGrid.CONE_9),
                TrajectoryLoader.getAutoPath("2 Piece N"),
                Commands.sequence(
                        RobotStates.initializeMechanisms(intake, manipulator, elevator, elbow).withTimeout(1),
                        fullAutoScoreWithMovement(GamePieceLocation.I1, drivetrain, intake,
                                manipulator, elevator,
                                elbow),
                        drivetrain.pathFollowingCommand(
                                TrajectoryLoader.getAutoPath("2 Piece N").getTrajectories().get(0)).asProxy()));
    }

    /**
     * Creates the 2 Piece N trajectory command.
     * 
     * @param autoPath   the {@link AutoPath} containing the Figure 8 trajectory.
     * @param drivetrain the drivetrain
     * @return the command
     */
    public static Supplier<AutoTrajectoryCommand> onePieceS(Drivetrain drivetrain, Intake intake,
            Manipulator manipulator, Elevator elevator, Elbow elbow) {
        return () -> new AutoTrajectoryCommand(getStartingPose(FieldConstants.Blue.LeftGrid.CONE_1),
                TrajectoryLoader.getAutoPath("2 Piece S"),
                Commands.sequence(
                        RobotStates.initializeMechanisms(intake, manipulator, elevator, elbow).withTimeout(1),
                        fullAutoScoreWithMovement(GamePieceLocation.A1, drivetrain, intake,
                                manipulator, elevator,
                                elbow),
                        drivetrain.pathFollowingCommand(
                                TrajectoryLoader.getAutoPath("2 Piece S").getTrajectories().get(0)).asProxy()));
    }

    /**
     * Creates the 2 Piece N trajectory command.
     * 
     * @param autoPath   the {@link AutoPath} containing the Figure 8 trajectory.
     * @param drivetrain the drivetrain
     * @return the command
     */
    public static Supplier<AutoTrajectoryCommand> twoPieceN(AutoPath autoPath, Drivetrain drivetrain, Intake intake,
            Manipulator manipulator, Elevator elevator, Elbow elbow) {
        return () -> new AutoTrajectoryCommand(getStartingPose(FieldConstants.Blue.RightGrid.CONE_9), autoPath,
                Commands.sequence(
                        fullAutoScoreWithMovement(GamePieceLocation.I1, drivetrain, intake,
                                manipulator, elevator,
                                elbow),
                        Commands.parallel(
                                drivetrain.pathFollowingCommand(
                                        autoPath.getTrajectories().get(0)).asProxy(),
                                fullAutoIntakeWithMovement(GamePiece.CUBE, drivetrain, intake, manipulator, elevator,
                                        elbow)),
                        fullAutoScoreWithMovement(new PathConstraints(1, 1), GamePieceLocation.H1, drivetrain, intake,
                                manipulator, elevator,
                                elbow)));
    }

    /**
     * Creates the 2 Piece Charge N trajectory command.
     * 
     * @param autoPath   the {@link AutoPath} containing the Figure 8 trajectory.
     * @param drivetrain the drivetrain
     * @return the command
     */
    public static Supplier<AutoTrajectoryCommand> twoPieceChargeN(AutoPath autoPath, Drivetrain drivetrain,
            Intake intake,
            Manipulator manipulator, Elevator elevator, Elbow elbow) {
        return () -> new AutoTrajectoryCommand(getStartingPose(FieldConstants.Blue.RightGrid.CONE_9), autoPath,
                Commands.sequence(
                        fullAutoScoreWithMovement(GamePieceLocation.I1, drivetrain, intake, manipulator, elevator,
                                elbow),
                        drivetrain.pathFollowingCommand(
                                autoPath.getTrajectories().get(0)).asProxy(),
                        fullAutoIntakeWithMovement(GamePiece.CUBE, drivetrain, intake, manipulator, elevator, elbow),
                        fullAutoScoreWithMovement(GamePieceLocation.H1, drivetrain, intake, manipulator, elevator,
                                elbow),
                        drivetrain.pathFollowingCommand(
                                autoPath.getTrajectories().get(2)).asProxy(),
                        drivetrain.chargeStationBalanceCommand().asProxy()));
    }

    /**
     * Creates the 3 Piece N trajectory command.
     * 
     * @param autoPath   the {@link AutoPath} containing the Figure 8 trajectory.
     * @param drivetrain the drivetrain
     * @return the command
     */
    public static Supplier<AutoTrajectoryCommand> threePieceN(AutoPath autoPath, Drivetrain drivetrain, Intake intake,
            Manipulator manipulator, Elevator elevator, Elbow elbow) {
        return () -> new AutoTrajectoryCommand(getStartingPose(FieldConstants.Blue.RightGrid.CONE_9), autoPath,
                Commands.sequence(
                        fullAutoScoreWithMovement(GamePieceLocation.I1, drivetrain, intake, manipulator, elevator,
                                elbow),
                        drivetrain.pathFollowingCommand(
                                autoPath.getTrajectories().get(0)).asProxy(),
                        fullAutoIntakeWithMovement(GamePiece.CUBE, drivetrain, intake, manipulator, elevator, elbow),
                        fullAutoScoreWithMovement(GamePieceLocation.H1, drivetrain, intake, manipulator, elevator,
                                elbow),
                        drivetrain.pathFollowingCommand(
                                autoPath.getTrajectories().get(2)).asProxy(),
                        fullAutoIntakeWithMovement(GamePiece.CONE, drivetrain, intake, manipulator, elevator, elbow),
                        fullAutoScoreWithMovement(GamePieceLocation.G1, drivetrain, intake, manipulator, elevator,
                                elbow)));
    }

    /**
     * Creates the 2 Piece S trajectory command.
     * 
     * @param autoPath   the {@link AutoPath} containing the Figure 8 trajectory.
     * @param drivetrain the drivetrain
     * @return the command
     */
    public static Supplier<AutoTrajectoryCommand> twoPieceS(AutoPath autoPath, Drivetrain drivetrain, Intake intake,
            Manipulator manipulator, Elevator elevator, Elbow elbow) {
        return () -> new AutoTrajectoryCommand(getStartingPose(FieldConstants.Blue.LeftGrid.CONE_1), autoPath,
                Commands.sequence(
                        fullAutoScoreWithMovement(GamePieceLocation.A1, drivetrain, intake, manipulator, elevator,
                                elbow),
                        drivetrain.pathFollowingCommand(
                                autoPath.getTrajectories().get(0)).asProxy(),
                        fullAutoIntakeWithMovement(GamePiece.CUBE, drivetrain, intake, manipulator, elevator, elbow),
                        fullAutoScoreWithMovement(GamePieceLocation.B1, drivetrain, intake, manipulator, elevator,
                                elbow)));
    }

    /**
     * Creates the 2 Piece S trajectory command.
     * 
     * @param autoPath   the {@link AutoPath} containing the Figure 8 trajectory.
     * @param drivetrain the drivetrain
     * @return the command
     */
    public static Supplier<AutoTrajectoryCommand> twoPieceChargeS(AutoPath autoPath, Drivetrain drivetrain,
            Intake intake,
            Manipulator manipulator, Elevator elevator, Elbow elbow) {
        return () -> new AutoTrajectoryCommand(getStartingPose(FieldConstants.Blue.LeftGrid.CONE_1), autoPath,
                Commands.sequence(
                        fullAutoScoreWithMovement(GamePieceLocation.A1, drivetrain, intake, manipulator, elevator,
                                elbow),
                        drivetrain.pathFollowingCommand(
                                autoPath.getTrajectories().get(0)).asProxy(),
                        fullAutoIntakeWithMovement(GamePiece.CUBE, drivetrain, intake, manipulator, elevator, elbow),
                        fullAutoScoreWithMovement(GamePieceLocation.B1, drivetrain, intake, manipulator, elevator,
                                elbow),
                        drivetrain.pathFollowingCommand(
                                autoPath.getTrajectories().get(2)).asProxy(),
                        drivetrain.chargeStationBalanceCommand().asProxy()));
    }

    /**
     * Creates the 3 Piece S trajectory command.
     * 
     * @param autoPath   the {@link AutoPath} containing the Figure 8 trajectory.
     * @param drivetrain the drivetrain
     * @return the command
     */
    public static Supplier<AutoTrajectoryCommand> threePieceS(AutoPath autoPath, Drivetrain drivetrain, Intake intake,
            Manipulator manipulator, Elevator elevator, Elbow elbow) {
        return () -> new AutoTrajectoryCommand(getStartingPose(FieldConstants.Blue.LeftGrid.CONE_1), autoPath,
                Commands.sequence(
                        fullAutoScoreWithMovement(GamePieceLocation.A1, drivetrain, intake, manipulator, elevator,
                                elbow),
                        drivetrain.pathFollowingCommand(
                                autoPath.getTrajectories().get(0)).asProxy(),
                        fullAutoIntakeWithMovement(GamePiece.CUBE, drivetrain, intake, manipulator, elevator, elbow),
                        fullAutoScoreWithMovement(GamePieceLocation.B1, drivetrain, intake, manipulator, elevator,
                                elbow),
                        drivetrain.pathFollowingCommand(
                                autoPath.getTrajectories().get(2)).asProxy(),
                        fullAutoIntakeWithMovement(GamePiece.CONE, drivetrain, intake, manipulator, elevator, elbow),
                        fullAutoScoreWithMovement(GamePieceLocation.C1, drivetrain, intake, manipulator, elevator,
                                elbow)));
    }

}
