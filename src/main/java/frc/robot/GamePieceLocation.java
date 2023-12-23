package frc.robot;

import java.util.HashMap;
import java.util.Objects;

import frc.robot.GamePieceLocation.GamePiece;
import frc.robot.GamePieceLocation.Grid;
import frc.robot.GamePieceLocation.GridPosition;
import frc.robot.GamePieceLocation.Level;

class GamePieceLocationMetadata {
    protected final GamePiece gamePiece;
    protected final Level level;
    protected final Grid grid;
    protected final GridPosition gridPosition;
    private int hashCode;

    public GamePieceLocationMetadata(GamePiece gamePiece, Level level, Grid grid, GridPosition gridPosition) {
        this.gamePiece = gamePiece;
        this.level = level;
        this.grid = grid;
        this.gridPosition = gridPosition;
        this.hashCode = Objects.hash(gamePiece, level, grid, gridPosition);
    }

    @Override
    public boolean equals(Object other) {
        GamePieceLocationMetadata otherMetadata = (GamePieceLocationMetadata) other;
        return this.gamePiece.equals(otherMetadata.gamePiece)
                && this.level.equals(otherMetadata.level)
                && this.grid.equals(otherMetadata.grid)
                && this.gridPosition.equals(otherMetadata.gridPosition);
    }

    @Override
    public int hashCode() {
        return this.hashCode;
    }
}

public enum GamePieceLocation {
    A1(new GamePieceLocationMetadata(GamePiece.CONE, Level.HIGH, Grid.LEFT, GridPosition.LEFT)),
    B1(new GamePieceLocationMetadata(GamePiece.CUBE, Level.HIGH, Grid.LEFT, GridPosition.MIDDLE)),
    C1(new GamePieceLocationMetadata(GamePiece.CONE, Level.HIGH, Grid.LEFT, GridPosition.RIGHT)),

    D1(new GamePieceLocationMetadata(GamePiece.CONE, Level.HIGH, Grid.MIDDLE, GridPosition.LEFT)),
    E1(new GamePieceLocationMetadata(GamePiece.CUBE, Level.HIGH, Grid.MIDDLE, GridPosition.MIDDLE)),
    F1(new GamePieceLocationMetadata(GamePiece.CONE, Level.HIGH, Grid.MIDDLE, GridPosition.RIGHT)),

    G1(new GamePieceLocationMetadata(GamePiece.CONE, Level.HIGH, Grid.RIGHT, GridPosition.LEFT)),
    H1(new GamePieceLocationMetadata(GamePiece.CUBE, Level.HIGH, Grid.RIGHT, GridPosition.MIDDLE)),
    I1(new GamePieceLocationMetadata(GamePiece.CONE, Level.HIGH, Grid.RIGHT, GridPosition.RIGHT)),

    A2(new GamePieceLocationMetadata(GamePiece.CONE, Level.MIDDLE, Grid.LEFT, GridPosition.LEFT)),
    B2(new GamePieceLocationMetadata(GamePiece.CUBE, Level.MIDDLE, Grid.LEFT, GridPosition.MIDDLE)),
    C2(new GamePieceLocationMetadata(GamePiece.CONE, Level.MIDDLE, Grid.LEFT, GridPosition.RIGHT)),

    D2(new GamePieceLocationMetadata(GamePiece.CONE, Level.MIDDLE, Grid.MIDDLE, GridPosition.LEFT)),
    E2(new GamePieceLocationMetadata(GamePiece.CUBE, Level.MIDDLE, Grid.MIDDLE, GridPosition.MIDDLE)),
    F2(new GamePieceLocationMetadata(GamePiece.CONE, Level.MIDDLE, Grid.MIDDLE, GridPosition.RIGHT)),

    G2(new GamePieceLocationMetadata(GamePiece.CONE, Level.MIDDLE, Grid.RIGHT, GridPosition.LEFT)),
    H2(new GamePieceLocationMetadata(GamePiece.CUBE, Level.MIDDLE, Grid.RIGHT, GridPosition.MIDDLE)),
    I2(new GamePieceLocationMetadata(GamePiece.CONE, Level.MIDDLE, Grid.RIGHT, GridPosition.RIGHT)),

    A3(new GamePieceLocationMetadata(GamePiece.HYBRID, Level.HYBRID, Grid.LEFT, GridPosition.LEFT)),
    B3(new GamePieceLocationMetadata(GamePiece.HYBRID, Level.HYBRID, Grid.LEFT, GridPosition.MIDDLE)),
    C3(new GamePieceLocationMetadata(GamePiece.HYBRID, Level.HYBRID, Grid.LEFT, GridPosition.RIGHT)),

    D3(new GamePieceLocationMetadata(GamePiece.HYBRID, Level.HYBRID, Grid.MIDDLE, GridPosition.LEFT)),
    E3(new GamePieceLocationMetadata(GamePiece.HYBRID, Level.HYBRID, Grid.MIDDLE, GridPosition.MIDDLE)),
    F3(new GamePieceLocationMetadata(GamePiece.HYBRID, Level.HYBRID, Grid.MIDDLE, GridPosition.RIGHT)),

    G3(new GamePieceLocationMetadata(GamePiece.HYBRID, Level.HYBRID, Grid.RIGHT, GridPosition.LEFT)),
    H3(new GamePieceLocationMetadata(GamePiece.HYBRID, Level.HYBRID, Grid.RIGHT, GridPosition.MIDDLE)),
    I3(new GamePieceLocationMetadata(GamePiece.HYBRID, Level.HYBRID, Grid.RIGHT, GridPosition.RIGHT));

    public final GamePiece gamePiece;
    public final Level level;
    public final Grid grid;
    public final GridPosition gridPosition;
    public final GamePieceLocationMetadata metadata;
    public static final HashMap<GamePieceLocationMetadata, GamePieceLocation> reverseMap = new HashMap<GamePieceLocationMetadata, GamePieceLocation>();

    static {
        for (GamePieceLocation e : values()) {
            reverseMap.put(e.metadata, e);
        }
    }

    public enum GamePiece {
        CONE,
        CUBE,
        HYBRID;
    }

    public enum Level {
        HYBRID,
        MIDDLE,
        HIGH;
    }

    public enum Grid {
        LEFT,
        MIDDLE,
        RIGHT;

        @Override
        public String toString() {
            return super.toString();
        }
    }

    public enum GridPosition {
        LEFT,
        MIDDLE,
        RIGHT;
    }

    private GamePieceLocation(GamePieceLocationMetadata metadata) {
        this.metadata = metadata;
        this.gamePiece = metadata.gamePiece;
        this.level = metadata.level;
        this.grid = metadata.grid;
        this.gridPosition = metadata.gridPosition;

    }

    public static GamePieceLocation from(GamePiece gamePiece, Level level, Grid grid,
            GridPosition gridPosition) {

        GamePieceLocation loc = reverseMap.get(new GamePieceLocationMetadata(gamePiece, level, grid, gridPosition));
        return loc;
    }
}