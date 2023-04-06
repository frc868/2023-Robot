package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import java.util.ArrayList;
import java.util.Optional;

import com.techhounds.houndutil.houndlog.LogGroup;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.enums.LogLevel;
import com.techhounds.houndutil.houndlog.logitems.StringLogItem;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Controls.OperatorControls;
import frc.robot.GamePieceLocation.Grid;
import frc.robot.GamePieceLocation.GridPosition;

public class GridInterface {
    private Optional<GamePieceLocation.Grid> setGrid = Optional.empty();
    private Optional<GamePieceLocation> setLocation = Optional.empty();

    private ArrayList<GamePieceLocation> previousLocations = new ArrayList<GamePieceLocation>();

    private Optional<GenericHID[]> hids = Optional.empty();

    public GridInterface() {
        LoggingManager.getInstance().addGroup("GridInterface", new LogGroup(
                new StringLogItem("setGrid", () -> setGrid.isEmpty() ? "null" : setGrid.get().toString(),
                        LogLevel.MAIN),
                new StringLogItem("setLocation",
                        () -> setLocation.isEmpty() ? "null" : setLocation.get().toString(),
                        LogLevel.MAIN)));
    }

    public void setHIDOutputDevices(GenericHID hid1, GenericHID hid2) {
        this.hids = Optional.of(new GenericHID[] { hid1, hid2 });
    }

    public boolean setGrid(GamePieceLocation.Grid grid) {
        if (this.setLocation.isPresent()) {
            return false;
        }

        if (this.setGrid.isPresent()) {
            this.reset();
        }

        this.setGrid = Optional.of(grid);

        if (hids.isPresent()) {
            switch (grid) {
                case LEFT:
                    hids.orElseThrow()[OperatorControls.LEFT_GRID.hid].setOutput(OperatorControls.LEFT_GRID.button,
                            true);
                    break;
                case MIDDLE:
                    hids.orElseThrow()[OperatorControls.MIDDLE_GRID.hid].setOutput(OperatorControls.MIDDLE_GRID.button,
                            true);
                    break;
                case RIGHT:
                    hids.orElseThrow()[OperatorControls.RIGHT_GRID.hid].setOutput(OperatorControls.RIGHT_GRID.button,
                            true);
                    break;
            }
        }
        return true;
    }

    public boolean setLocation(GamePieceLocation.GamePiece gamePiece,
            GamePieceLocation.GridPosition gridPosition,
            GamePieceLocation.Level level) {
        if (setGrid.isEmpty()) {
            return false;
        }
        if (setLocation.isPresent()) {
            return false;
        }

        Grid grid = setGrid.orElseThrow();

        if (DriverStation.getAlliance() == Alliance.Blue) {
            GridPosition invertedGridPosition;
            if (gridPosition == GamePieceLocation.GridPosition.LEFT) {
                invertedGridPosition = GamePieceLocation.GridPosition.RIGHT;
            } else if (gridPosition == GamePieceLocation.GridPosition.RIGHT) {
                invertedGridPosition = GamePieceLocation.GridPosition.LEFT;
            } else {
                invertedGridPosition = gridPosition;
            }

            Grid invertedGrid;
            if (grid == GamePieceLocation.Grid.LEFT) {
                invertedGrid = GamePieceLocation.Grid.RIGHT;
            } else if (grid == GamePieceLocation.Grid.RIGHT) {
                invertedGrid = GamePieceLocation.Grid.LEFT;
            } else {
                invertedGrid = grid;
            }

            this.setLocation = Optional
                    .of(GamePieceLocation.from(gamePiece, level, invertedGrid, invertedGridPosition));
        } else {
            this.setLocation = Optional
                    .of(GamePieceLocation.from(gamePiece, level, grid, gridPosition));
        }

        if (hids.isPresent()) {
            switch (gridPosition) {
                case LEFT:
                    switch (level) {
                        case LOW:
                            hids.orElseThrow()[OperatorControls.GRIDPOS_C1.hid]
                                    .setOutput(OperatorControls.GRIDPOS_C1.button, true);
                            break;
                        case MIDDLE:
                            hids.orElseThrow()[OperatorControls.GRIDPOS_B1.hid]
                                    .setOutput(OperatorControls.GRIDPOS_B1.button, true);
                            break;
                        case HIGH:
                            hids.orElseThrow()[OperatorControls.GRIDPOS_A1.hid]
                                    .setOutput(OperatorControls.GRIDPOS_A1.button, true);
                            break;
                    }
                    break;
                case MIDDLE:
                    switch (level) {
                        case LOW:
                            hids.orElseThrow()[OperatorControls.GRIDPOS_C2.hid]
                                    .setOutput(OperatorControls.GRIDPOS_C2.button, true);
                            break;
                        case MIDDLE:
                            hids.orElseThrow()[OperatorControls.GRIDPOS_B2.hid]
                                    .setOutput(OperatorControls.GRIDPOS_B2.button, true);
                            break;
                        case HIGH:
                            hids.orElseThrow()[OperatorControls.GRIDPOS_A2.hid]
                                    .setOutput(OperatorControls.GRIDPOS_A2.button, true);
                            break;
                    }
                    break;
                case RIGHT:
                    switch (level) {
                        case LOW:
                            hids.orElseThrow()[OperatorControls.GRIDPOS_C3.hid]
                                    .setOutput(OperatorControls.GRIDPOS_C3.button, true);
                            break;
                        case MIDDLE:
                            hids.orElseThrow()[OperatorControls.GRIDPOS_B3.hid]
                                    .setOutput(OperatorControls.GRIDPOS_B3.button, true);
                            break;
                        case HIGH:
                            hids.orElseThrow()[OperatorControls.GRIDPOS_A3.hid]
                                    .setOutput(OperatorControls.GRIDPOS_A3.button, true);
                            break;
                    }
                    break;
            }
        }
        return true;
    }

    public CommandBase setGridCommand(GamePieceLocation.Grid grid) {
        return runOnce(() -> {
            setGrid(grid);
        }).ignoringDisable(true);
    }

    public CommandBase setLocationCommand(
            GamePieceLocation.GamePiece gamePiece,
            GamePieceLocation.GridPosition gridPosition,
            GamePieceLocation.Level level) {
        return runOnce(() -> {
            setLocation(gamePiece, gridPosition, level);
        }).ignoringDisable(true);
    }

    /**
     * Reset the game piece selector.
     */
    public void reset() {
        this.setGrid = Optional.empty();
        this.setLocation = Optional.empty();
        if (hids.isPresent()) {
            for (OperatorControls control : OperatorControls.getGridControls()) {
                hids.orElseThrow()[control.hid]
                        .setOutput(control.button, false);
            }
        }
    }

    /**
     * Run this whenever a game piece has been scored to add it to the registry and
     * clear the set game piece selection.
     */
    public void gamePieceScored() {
        previousLocations.add(setLocation.get());
        setGrid = Optional.empty();
        setLocation = Optional.empty();
    }

    public void sendToShuffleboard() {
        // TODO
    }

    public Optional<GamePieceLocation.Grid> getSetGrid() {
        return this.setGrid;
    }

    public Optional<GamePieceLocation> getSetLocation() {
        return this.setLocation;
    }

}
