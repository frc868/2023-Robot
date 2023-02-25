package frc.robot.commands;

import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.techhounds.houndutil.houndauto.AutoManager;
import com.techhounds.houndutil.houndauto.AutoPath;
import com.techhounds.houndutil.houndauto.AutoTrajectoryCommand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.GamePieceLocation;
import frc.robot.GamePieceLocation.GamePiece;
import frc.robot.GamePieceLocation.Level;
import frc.robot.commands.RobotStates.RobotState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Drivetrain.DriveMode;

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
        return Commands.sequence(
                RobotStates.autoDriveCommand(() -> RobotState.SCORING, () -> gamePieceLocation, drivetrain),
                Scoring.scoreGamePieceAutoCommand(() -> gamePieceLocation.gamePiece, () -> gamePieceLocation.level,
                        drivetrain, intake, manipulator,
                        elevator, elbow),
                RobotStates.stowElevatorCommand(intake, manipulator, elevator, elbow));
    }

    public static CommandBase fullAutoIntakeWithMovement(
            GamePiece gamePiece,
            Drivetrain drivetrain, Intake intake,
            Manipulator manipulator, Elevator elevator, Elbow elbow, LEDs leds) {
        return RobotStates.setIntakeModeCommand(gamePiece, leds).andThen(
                RobotStates.intakeGamePieceAutoCommand(intake, manipulator, elevator, elbow, leds).deadlineWith(
                        Commands.run(() -> drivetrain.drive(0.1, 0, 0, DriveMode.ROBOT_RELATIVE), drivetrain)
                                .asProxy()));
    }

    /**
     * Creates the 2 Piece N trajectory command.
     * 
     * @param autoPath   the {@link AutoPath} containing the Figure 8 trajectory.
     * @param drivetrain the drivetrain
     * @return the command
     */
    public static Supplier<AutoTrajectoryCommand> twoPieceN(AutoPath autoPath, Drivetrain drivetrain, Intake intake,
            Manipulator manipulator, Elevator elevator, Elbow elbow, LEDs leds) {
        return () -> new AutoTrajectoryCommand(autoPath,
                Commands.sequence(
                        RobotStates.autoDriveCommand(() -> RobotState.SCORING, () -> GamePieceLocation.I1, drivetrain),
                        Scoring.scoreGamePieceAutoCommand(() -> GamePiece.CONE, () -> Level.HIGH, drivetrain, intake,
                                manipulator, elevator, elbow),
                        Commands.parallel(
                                RobotStates.stowElevatorCommand(intake, manipulator, elevator, elbow),
                                drivetrain.pathFollowingCommand(
                                        autoPath.getTrajectories().get(0)).asProxy()),
                        fullAutoIntakeWithMovement(GamePiece.CONE, drivetrain, intake, manipulator, elevator, elbow,
                                leds),

                        // RobotStates.autoDrive(() -> RobotState.SCORING, () -> GamePieceLocation.G1,
                        // drivetrain),
                        Commands.parallel(
                                drivetrain.pathFollowingCommand(autoPath.getTrajectories().get(1)).asProxy(),
                                Commands.waitSeconds(1)
                                        .andThen(Scoring.scoreGamePieceAutoCommand(() -> GamePiece.CONE,
                                                () -> Level.HIGH,
                                                drivetrain,
                                                intake,
                                                manipulator, elevator, elbow))),
                        Commands.parallel(
                                RobotStates.stowElevatorCommand(intake, manipulator, elevator, elbow),
                                drivetrain.pathFollowingCommand(autoPath.getTrajectories().get(2)).asProxy()),
                        drivetrain.chargeStationBalanceCommand().asProxy()));
    }
}
