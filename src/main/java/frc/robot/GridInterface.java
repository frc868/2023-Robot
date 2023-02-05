package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import java.util.ArrayList;
import java.util.Optional;

import com.techhounds.houndutil.houndlog.LogGroup;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.enums.LogLevel;
import com.techhounds.houndutil.houndlog.logitems.StringLogItem;

import edu.wpi.first.wpilibj2.command.Command;

public class GridInterface {

    private Optional<GamePieceLocation.Grid> setGrid = Optional.empty();
    private Optional<GamePieceLocation> setLocation = Optional.empty();

    private ArrayList<GamePieceLocation> previousLocations = new ArrayList<GamePieceLocation>();

    public GridInterface() {
        LoggingManager.getInstance().addGroup("GridInterface", new LogGroup(
                new StringLogItem("setGrid", () -> setGrid.isEmpty() ? "null" : setGrid.get().toString(),
                        LogLevel.MAIN),
                new StringLogItem("setLocation",
                        () -> setLocation.isEmpty() ? "null" : setLocation.get().toString(),
                        LogLevel.MAIN)));
    }

    public boolean setGrid(GamePieceLocation.Grid grid) {
        this.setGrid = Optional.of(grid);
        return true;
    }

    public boolean setLocation(GamePieceLocation.GamePiece gamePiece,
            GamePieceLocation.GridPosition gridPosition,
            GamePieceLocation.Level level) {
        if (setGrid.isEmpty()) {
            return false;
        }

        this.setLocation = Optional
                .of(GamePieceLocation.from(gamePiece, level, setGrid.get(), gridPosition));
        return true;
    }

    public Command setGridCommand(GamePieceLocation.Grid grid) {
        return runOnce(() -> {
            setGrid(grid);
        });
    }

    public Command setLocationCommand(
            GamePieceLocation.GamePiece gamePiece,
            GamePieceLocation.GridPosition gridPosition,
            GamePieceLocation.Level level) {
        return runOnce(() -> {
            setLocation(gamePiece, gridPosition, level);
        });
    }

    /**
     * Reset the game piece selector.
     */
    public void reset() {
        this.setGrid = Optional.empty();
        this.setLocation = Optional.empty();
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
