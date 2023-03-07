package frc.robot;

import java.util.ArrayList;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.techhounds.houndutil.houndauto.AutoManager;
import com.techhounds.houndutil.houndauto.AutoPath;
import com.techhounds.houndutil.houndauto.AutoTrajectoryCommand;
import com.techhounds.houndutil.houndlog.LogGroup;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.loggers.SendableLogger;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.GamePieceLocation.GamePiece;
import frc.robot.GamePieceLocation.Grid;
import frc.robot.GamePieceLocation.GridPosition;
import frc.robot.GamePieceLocation.Level;
import frc.robot.commands.Autos;
import frc.robot.commands.RobotStates;
import frc.robot.commands.RobotStates.RobotState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Manipulator;

public class AutoGenerator {
    enum StartingLocation {
        North(new Pose2d(1.76, 5.06, Rotation2d.fromDegrees(180))),
        Center(new Pose2d(1.76, 2.75, Rotation2d.fromDegrees(180))),
        South(new Pose2d(1.76, 0.44, Rotation2d.fromDegrees(180)));

        public final Pose2d pose;

        private StartingLocation(Pose2d pose) {
            this.pose = pose;
        }
    }

    private SendableChooser<StartingLocation> startingLocationChooser = new SendableChooser<StartingLocation>();

    private SendableChooser<GamePiece> preloadGamePieceChooser = new SendableChooser<GamePiece>();
    private SendableChooser<Pair<Grid, GridPosition>> preloadGridPositionChooser = new SendableChooser<Pair<Grid, GridPosition>>();
    private SendableChooser<Level> preloadLevelChooser = new SendableChooser<Level>();
    private BooleanEntry enablePreload;

    private SendableChooser<GamePiece> secondGamePieceChooser = new SendableChooser<GamePiece>();
    private SendableChooser<Pair<Grid, GridPosition>> secondGridPositionChooser = new SendableChooser<Pair<Grid, GridPosition>>();
    private SendableChooser<Level> secondLevelChooser = new SendableChooser<Level>();
    private BooleanEntry enableSecond;
    private BooleanEntry holdSecond;

    private SendableChooser<GamePiece> thirdGamePieceChooser = new SendableChooser<GamePiece>();
    private SendableChooser<Pair<Grid, GridPosition>> thirdGridPositionChooser = new SendableChooser<Pair<Grid, GridPosition>>();
    private SendableChooser<Level> thirdLevelChooser = new SendableChooser<Level>();
    private BooleanEntry enableThird;
    private BooleanEntry holdThird;

    private BooleanEntry balanceOnChargeStation;

    private AutoTrajectoryCommand autoCommand;

    public AutoGenerator(Drivetrain drivetrain, Intake intake,
            Manipulator manipulator, Elevator elevator, Elbow elbow, LEDs leds) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("HoundLog/AutoGenerator");

        startingLocationChooser.addOption("North", StartingLocation.North);
        startingLocationChooser.addOption("Center", StartingLocation.Center);
        startingLocationChooser.addOption("South", StartingLocation.South);

        enablePreload = table.getBooleanTopic("Enable Preload").getEntry(false);
        enablePreload.set(false);
        enableSecond = table.getBooleanTopic("Enable Second Piece").getEntry(false);
        enableSecond.set(false);
        enableThird = table.getBooleanTopic("Enable Third Piece").getEntry(false);
        enableThird.set(false);

        holdSecond = table.getBooleanTopic("Hold Second Piece").getEntry(false);
        holdSecond.set(false);
        holdThird = table.getBooleanTopic("Hold Third Piece").getEntry(false);
        holdThird.set(false);

        balanceOnChargeStation = table.getBooleanTopic("Balance on Charge Station").getEntry(false);
        balanceOnChargeStation.set(false);

        configChooser(preloadGamePieceChooser, preloadGridPositionChooser, preloadLevelChooser);
        configChooser(secondGamePieceChooser, secondGridPositionChooser, secondLevelChooser);
        configChooser(thirdGamePieceChooser, thirdGridPositionChooser, thirdLevelChooser);

        LoggingManager.getInstance().addGroup("AutoGenerator", new LogGroup(
                new SendableLogger("Starting Location", startingLocationChooser),
                new SendableLogger("Preload Game Piece", preloadGamePieceChooser),
                new SendableLogger("Preload Grid Position", preloadGridPositionChooser),
                new SendableLogger("Preload Level", preloadLevelChooser),
                new SendableLogger("Second Game Piece", secondGamePieceChooser),
                new SendableLogger("Second Grid Position", secondGridPositionChooser),
                new SendableLogger("Second Level", secondLevelChooser),
                new SendableLogger("Third Game Piece", thirdGamePieceChooser),
                new SendableLogger("Third Grid Position", thirdGridPositionChooser),
                new SendableLogger("Third Level", thirdLevelChooser),
                new SendableLogger("Build", build(drivetrain, intake, manipulator, elevator, elbow, leds))));
    }

    private void configChooser(SendableChooser<GamePiece> gamePieceChooser,
            SendableChooser<Pair<Grid, GridPosition>> gridPositionChooser, SendableChooser<Level> levelChooser) {

        gamePieceChooser.addOption("Cone", GamePiece.CONE);
        gamePieceChooser.addOption("Cube", GamePiece.CUBE);

        gridPositionChooser.addOption("A",
                new Pair<Grid, GridPosition>(Grid.LEFT, GridPosition.LEFT));
        gridPositionChooser.addOption("B",
                new Pair<Grid, GridPosition>(Grid.LEFT, GridPosition.MIDDLE));
        gridPositionChooser.addOption("C",
                new Pair<Grid, GridPosition>(Grid.LEFT, GridPosition.RIGHT));
        gridPositionChooser.addOption("D",
                new Pair<Grid, GridPosition>(Grid.MIDDLE, GridPosition.LEFT));
        gridPositionChooser.addOption("E",
                new Pair<Grid, GridPosition>(Grid.MIDDLE, GridPosition.MIDDLE));
        gridPositionChooser.addOption("F",
                new Pair<Grid, GridPosition>(Grid.MIDDLE, GridPosition.RIGHT));
        gridPositionChooser.addOption("G",
                new Pair<Grid, GridPosition>(Grid.RIGHT, GridPosition.LEFT));
        gridPositionChooser.addOption("H",
                new Pair<Grid, GridPosition>(Grid.RIGHT, GridPosition.MIDDLE));
        gridPositionChooser.addOption("I",
                new Pair<Grid, GridPosition>(Grid.RIGHT, GridPosition.RIGHT));

        levelChooser.addOption("High", Level.HIGH);
        levelChooser.addOption("Middle", Level.MIDDLE);
        levelChooser.addOption("Low", Level.LOW);
    }

    public void createCommand(Drivetrain drivetrain, Intake intake,
            Manipulator manipulator, Elevator elevator, Elbow elbow, LEDs leds) {

        SequentialCommandGroup commandGroup = new SequentialCommandGroup();
        ArrayList<PathPlannerTrajectory> trajectories = new ArrayList<PathPlannerTrajectory>();

        System.out.println(enablePreload.get());

        StartingLocation startingLocation = startingLocationChooser.getSelected();
        if (startingLocation != null) {
            // this.startingLocation = startingLocation.pose;
        } else {
            return;
        }

        if (enablePreload.get()
                && preloadGamePieceChooser.getSelected() != null
                && preloadLevelChooser.getSelected() != null
                && preloadGridPositionChooser.getSelected() != null) {

            GamePieceLocation preload = GamePieceLocation.from(preloadGamePieceChooser.getSelected(),
                    preloadLevelChooser.getSelected(), preloadGridPositionChooser.getSelected().getFirst(),
                    preloadGridPositionChooser.getSelected().getSecond());

            if (preload != null) {
                commandGroup.addCommands(
                        Autos.fullAutoScoreWithMovement(preload, drivetrain, intake, manipulator, elevator, elbow));
                trajectories.add(RobotStates.getAutoDriveTraj(() -> RobotState.SCORING, () -> preload, drivetrain));
            }
        }
        if (enableSecond.get()
                && secondGamePieceChooser.getSelected() != null
                && secondLevelChooser.getSelected() != null
                && secondGridPositionChooser.getSelected() != null) {
            GamePieceLocation second = GamePieceLocation.from(secondGamePieceChooser.getSelected(),
                    secondLevelChooser.getSelected(), secondGridPositionChooser.getSelected().getFirst(),
                    secondGridPositionChooser.getSelected().getSecond());

            if (second != null) {
                commandGroup.addCommands(
                        Autos.fullAutoScoreWithMovement(second, drivetrain, intake, manipulator, elevator, elbow));
                trajectories.add(RobotStates.getAutoDriveTraj(() -> RobotState.SCORING, () -> second, drivetrain));
            }
        }
        if (enableThird.get()
                && thirdGamePieceChooser.getSelected() != null
                && thirdLevelChooser.getSelected() != null
                && thirdGridPositionChooser.getSelected() != null) {
            GamePieceLocation third = GamePieceLocation.from(thirdGamePieceChooser.getSelected(),
                    thirdLevelChooser.getSelected(), thirdGridPositionChooser.getSelected().getFirst(),
                    thirdGridPositionChooser.getSelected().getSecond());

            if (third != null) {
                commandGroup.addCommands(
                        Autos.fullAutoScoreWithMovement(third, drivetrain, intake, manipulator, elevator, elbow));
                trajectories.add(RobotStates.getAutoDriveTraj(() -> RobotState.SCORING, () -> third, drivetrain));
            }
        }

        if (trajectories.size() > 0) {
            System.out.println(trajectories.size());
            this.autoCommand = new AutoTrajectoryCommand(startingLocation.pose,
                    new AutoPath("AutoGenerator", trajectories), commandGroup);
        } else {
            this.autoCommand = new AutoTrajectoryCommand(startingLocation.pose, commandGroup);
        }

        // this.trajectory = trajectories.get(0);
    }

    public CommandBase build(Drivetrain drivetrain, Intake intake,
            Manipulator manipulator, Elevator elevator, Elbow elbow, LEDs leds) {
        return Commands.runOnce(() -> createCommand(drivetrain, intake, manipulator, elevator, elbow, leds))
                .andThen(() -> AutoManager.getInstance().updateShuffleboard(true))
                .withName("Build").ignoringDisable(true);
    }

    public Supplier<AutoTrajectoryCommand> getAutoCommand() {
        return () -> autoCommand == null ? new AutoTrajectoryCommand(new Pose2d(), Commands.none())
                : autoCommand;
    }

    public CommandBase preset() {
        return Commands.runOnce(() -> {
            NetworkTableInstance.getDefault().getTable("HoundLog/AutoGenerator/Second Grid Position")
                    .getStringTopic("selected").publish().set("A");
            // NetworkTableInstance.getDefault().getTable("HoundLog/AutoGenerator/Second
            // Grid Position")
            // .getStringTopic("active").publish().set("A");
            // NetworkTableInstance.getDefault().getTable("HoundLog/AutoGenerator/Second
            // Grid Position")
            // .getStringTopic("default").publish().set("A");
            // secondGridPositionChooser.setDefaultOption("A", new Pair<Grid,
            // GridPosition>(Grid.LEFT, GridPosition.LEFT));\
        }).ignoringDisable(true);
    }

}
