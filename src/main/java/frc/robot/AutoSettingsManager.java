package frc.robot;

import java.util.Map;
import java.util.Optional;
import java.util.function.Function;

import com.techhounds.houndutil.houndlog.LogGroup;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.loggers.SendableLogger;

import edu.wpi.first.math.Pair;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.GamePieceLocation.GamePiece;
import frc.robot.GamePieceLocation.Grid;
import frc.robot.GamePieceLocation.GridPosition;
import frc.robot.GamePieceLocation.Level;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Manipulator;

public class AutoSettingsManager {
    public class AutoSettings {
        public boolean enableCubapult;

        public Optional<GamePieceLocation> preloadGamePieceLocation = Optional.empty();
        public boolean enablePreload;

        public Optional<GamePieceLocation> secondGamePieceLocation = Optional.empty();
        public boolean enableSecond;

        public Optional<GamePieceLocation> thirdGamePieceLocation = Optional.empty();
        public boolean enableThird;

        public boolean balanceOnChargeStation;

        public AutoSettings(
                boolean enableCubapult,
                GamePiece preloadGamePiece, Grid preloadGrid, GridPosition preloadGridPosition, Level preloadLevel,
                boolean enablePreload,
                GamePiece secondGamePiece, Grid secondGrid, GridPosition secondGridPosition, Level secondLevel,
                boolean enableSecond,
                GamePiece thirdGamePiece, Grid thirdGrid, GridPosition thirdGridPosition, Level thirdLevel,
                boolean enableThird,
                boolean balanceOnChargeStation) {
            this.enableCubapult = enableCubapult;

            this.preloadGamePieceLocation = Optional
                    .ofNullable(
                            GamePieceLocation.from(preloadGamePiece, preloadLevel, preloadGrid, preloadGridPosition));
            this.enablePreload = enablePreload;
            this.secondGamePieceLocation = Optional
                    .ofNullable(GamePieceLocation.from(secondGamePiece, secondLevel, secondGrid, secondGridPosition));
            this.enableSecond = enableSecond;
            this.thirdGamePieceLocation = Optional
                    .ofNullable(GamePieceLocation.from(thirdGamePiece, thirdLevel, thirdGrid, thirdGridPosition));
            this.enableThird = enableThird;

            this.balanceOnChargeStation = balanceOnChargeStation;
        }

        public AutoSettings(
                boolean enableCubapult,
                GamePieceLocation preloadGamePieceLocation,
                boolean enablePreload,
                GamePieceLocation secondGamePieceLocation,
                boolean enableSecond,
                GamePieceLocation thirdGamePieceLocation,
                boolean enableThird,
                boolean balanceOnChargeStation) {
            this.enableCubapult = enableCubapult;

            this.preloadGamePieceLocation = Optional.ofNullable(preloadGamePieceLocation);
            this.enablePreload = enablePreload;
            this.secondGamePieceLocation = Optional.ofNullable(secondGamePieceLocation);
            this.enableSecond = enableSecond;
            this.thirdGamePieceLocation = Optional.ofNullable(thirdGamePieceLocation);
            this.enableThird = enableThird;

            this.balanceOnChargeStation = balanceOnChargeStation;
        }
    }

    private BooleanEntry enableCubapult;

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

    private AutoSettings previousPreset = null;
    private SendableChooser<AutoSettings> presetChooser = new SendableChooser<AutoSettings>();

    public AutoSettingsManager(Drivetrain drivetrain, Intake intake,
            Manipulator manipulator, Elevator elevator, Elbow elbow, LEDs leds) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("HoundLog/AutoSettings");
        enableCubapult = table.getBooleanTopic("Enable Cubapult").getEntry(false);

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

        presetChooser.addOption("North Link Cone",
                new AutoSettings(true,
                        null, false,
                        GamePieceLocation.I1, true,
                        GamePieceLocation.G1, true,
                        false));
        presetChooser.addOption("North Link Cube",
                new AutoSettings(true,
                        GamePieceLocation.H1, false,
                        GamePieceLocation.H1, true,
                        GamePieceLocation.H2, true,
                        false));

        new Trigger(() -> presetChooser.getSelected() != previousPreset).onTrue(Commands.runOnce(() -> {
            previousPreset = presetChooser.getSelected();
            displayPreset(previousPreset);
        }).ignoringDisable(true));

        LoggingManager.getInstance().addGroup("AutoSettings", new LogGroup(
                new SendableLogger("Preload Game Piece", preloadGamePieceChooser),
                new SendableLogger("Preload Grid Position", preloadGridPositionChooser),
                new SendableLogger("Preload Level", preloadLevelChooser),
                new SendableLogger("Second Game Piece", secondGamePieceChooser),
                new SendableLogger("Second Grid Position", secondGridPositionChooser),
                new SendableLogger("Second Level", secondLevelChooser),
                new SendableLogger("Third Game Piece", thirdGamePieceChooser),
                new SendableLogger("Third Grid Position", thirdGridPositionChooser),
                new SendableLogger("Third Level", thirdLevelChooser),
                new SendableLogger("Preset", presetChooser)));
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

    public AutoSettings getSettings() {
        return new AutoSettings(
                enableCubapult.get(),

                preloadGamePieceChooser.getSelected(),
                preloadGridPositionChooser.getSelected() != null ? preloadGridPositionChooser.getSelected().getFirst()
                        : null,
                preloadGridPositionChooser.getSelected() != null ? preloadGridPositionChooser.getSelected().getSecond()
                        : null,
                preloadLevelChooser.getSelected(),
                enablePreload.get(),

                secondGamePieceChooser.getSelected(),
                secondGridPositionChooser.getSelected() != null ? secondGridPositionChooser.getSelected().getFirst()
                        : null,
                secondGridPositionChooser.getSelected() != null ? secondGridPositionChooser.getSelected().getSecond()
                        : null,
                secondLevelChooser.getSelected(),
                enableSecond.get(),

                thirdGamePieceChooser.getSelected(),
                thirdGridPositionChooser.getSelected() != null ? thirdGridPositionChooser.getSelected().getFirst()
                        : null,
                thirdGridPositionChooser.getSelected() != null ? thirdGridPositionChooser.getSelected().getSecond()
                        : null,
                thirdLevelChooser.getSelected(),
                enableThird.get(),

                balanceOnChargeStation.get());
    }

    private void displayPreset(AutoSettings settings) {
        Function<GamePieceLocation, String> gridPosString = (GamePieceLocation loc) -> {
            switch (loc.grid) {
                case LEFT:
                    switch (loc.gridPosition) {
                        case LEFT:
                            return "A";
                        case MIDDLE:
                            return "B";
                        case RIGHT:
                            return "C";
                        default:
                            return "";
                    }
                case MIDDLE:
                    switch (loc.gridPosition) {
                        case LEFT:
                            return "D";
                        case MIDDLE:
                            return "E";
                        case RIGHT:
                            return "F";
                        default:
                            return "";
                    }
                case RIGHT:
                    switch (loc.gridPosition) {
                        case LEFT:
                            return "G";
                        case MIDDLE:
                            return "H";
                        case RIGHT:
                            return "I";
                        default:
                            return "";
                    }
                default:
                    return "";
            }
        };

        Function<GamePieceLocation, String> levelString = (GamePieceLocation loc) -> {
            Map<Level, String> levelMap = Map
                    .of(
                            Level.HIGH, "High",
                            Level.MIDDLE, "Middle",
                            Level.LOW, "Low");

            return levelMap.get(loc.level);
        };

        enableCubapult.set(settings.enableCubapult);
        try {
            enablePreload.set(settings.enablePreload);
            NetworkTableInstance.getDefault().getTable("HoundLog/AutoSettings/Preload Game Piece").getEntry("selected")
                    .setString(settings.preloadGamePieceLocation.get().gamePiece == GamePiece.CONE ? "Cone" : "Cube");
            NetworkTableInstance.getDefault().getTable("HoundLog/AutoSettings/Preload Grid Position")
                    .getEntry("selected")
                    .setString(gridPosString.apply(settings.preloadGamePieceLocation.get()));
            NetworkTableInstance.getDefault().getTable("HoundLog/AutoSettings/Preload Level").getEntry("selected")
                    .setString(levelString.apply(settings.preloadGamePieceLocation.get()));
        } catch (Exception e) {
        }

        try {
            enableSecond.set(settings.enableSecond);
            NetworkTableInstance.getDefault().getTable("HoundLog/AutoSettings/Second Game Piece").getEntry("selected")
                    .setString(settings.secondGamePieceLocation.get().gamePiece == GamePiece.CONE ? "Cone" : "Cube");
            NetworkTableInstance.getDefault().getTable("HoundLog/AutoSettings/Second Grid Position")
                    .getEntry("selected")
                    .setString(gridPosString.apply(settings.secondGamePieceLocation.get()));
            NetworkTableInstance.getDefault().getTable("HoundLog/AutoSettings/Second Level").getEntry("selected")
                    .setString(levelString.apply(settings.secondGamePieceLocation.get()));
        } catch (Exception e) {
        }

        enableThird.set(settings.enableThird);
        try {
            NetworkTableInstance.getDefault().getTable("HoundLog/AutoSettings/Third Game Piece").getEntry("selected")
                    .setString(settings.thirdGamePieceLocation.get().gamePiece == GamePiece.CONE ? "Cone" : "Cube");
            NetworkTableInstance.getDefault().getTable("HoundLog/AutoSettings/Third Grid Position").getEntry("selected")
                    .setString(gridPosString.apply(settings.thirdGamePieceLocation.get()));
            NetworkTableInstance.getDefault().getTable("HoundLog/AutoSettings/Third Level").getEntry("selected")
                    .setString(levelString.apply(settings.thirdGamePieceLocation.get()));
        } catch (Exception e) {
        }

    }

}
