package frc.robot;

import com.techhounds.houndutil.houndlog.LogGroup;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.loggers.SendableLogger;

import frc.robot.Modes.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.GamePieceLocation.GamePiece;
import frc.robot.GamePieceLocation.Grid;
import frc.robot.GamePieceLocation.GridPosition;
import frc.robot.GamePieceLocation.Level;
import frc.robot.commands.IntakingCommands;
import frc.robot.commands.ScoringCommands;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Elbow.ElbowPosition;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorPosition;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LEDs.LEDState;
import frc.robot.subsystems.Manipulator;

public class NTCommandManager {
    private static final SendableChooser<ElevatorPosition> elevatorPositionChooser = new SendableChooser<ElevatorPosition>();
    private static final SendableChooser<ElbowPosition> elbowPositionChooser = new SendableChooser<ElbowPosition>();
    private static final SendableChooser<LEDState> ledStateChooser = new SendableChooser<LEDState>();
    private static final SendableChooser<GamePiece> gamePieceChooser = new SendableChooser<GamePiece>();
    private static final SendableChooser<Level> levelChooser = new SendableChooser<Level>();
    private static final SendableChooser<RobotState> robotStateChooser = new SendableChooser<RobotState>();

    private static boolean secondaryButton = false;

    public static void configureNTCommands(Drivetrain drivetrain,
            GridInterface gridInterface, Intake intake,
            Manipulator manipulator, Elevator elevator, Elbow elbow, LEDs leds) {
        if (Constants.IS_VIRTUAL_BUTTON_PANEL_ENABLED) {
            LoggingManager.getInstance().addGroup("Commands/Operator Panel", new LogGroup(
                    new SendableLogger("Select Left Grid",
                            gridInterface.setGridCommand(Grid.LEFT).withName("Left Grid")),
                    new SendableLogger("Select Middle Grid",
                            gridInterface.setGridCommand(Grid.MIDDLE).withName("Middle Grid")),
                    new SendableLogger("Select Right Grid",
                            gridInterface.setGridCommand(Grid.RIGHT).withName("Right Grid")),
                    new SendableLogger("Select Left High Location",
                            gridInterface.setLocationCommand(GamePiece.CONE, GridPosition.LEFT, Level.HIGH)
                                    .withName("Left High Location")),
                    new SendableLogger("Select Middle High Location",
                            gridInterface.setLocationCommand(GamePiece.CUBE, GridPosition.MIDDLE, Level.HIGH)
                                    .withName("Middle High Location")),
                    new SendableLogger("Select Right High Location",
                            gridInterface.setLocationCommand(GamePiece.CONE, GridPosition.RIGHT, Level.HIGH)
                                    .withName("Right High Location")),
                    new SendableLogger("Select Left Middle Location",
                            gridInterface.setLocationCommand(GamePiece.CONE, GridPosition.LEFT, Level.MIDDLE)
                                    .withName("Left Middle Location")),
                    new SendableLogger("Select Middle Middle Location",
                            gridInterface.setLocationCommand(GamePiece.CUBE, GridPosition.MIDDLE, Level.MIDDLE)
                                    .withName("Middle Middle Location")),
                    new SendableLogger("Select Right Middle Location",
                            gridInterface.setLocationCommand(GamePiece.CONE, GridPosition.RIGHT, Level.MIDDLE)
                                    .withName("Right Middle Location")),
                    new SendableLogger("Select Left Low Location",
                            gridInterface.setLocationCommand(GamePiece.HYBRID, GridPosition.LEFT, Level.LOW)
                                    .withName("Left Low Location")),
                    new SendableLogger("Select Middle Low Location",
                            gridInterface.setLocationCommand(GamePiece.HYBRID, GridPosition.MIDDLE, Level.LOW)
                                    .withName("Middle Low Location")),
                    new SendableLogger("Select Right Low Location",
                            gridInterface.setLocationCommand(GamePiece.HYBRID, GridPosition.RIGHT, Level.LOW)
                                    .withName("Right Low Location")),
                    new SendableLogger("Reset",
                            Commands.runOnce(() -> gridInterface.reset()).withName("Reset")),
                    new SendableLogger("Score",
                            ScoringCommands
                                    .fullScoreSequenceCommand(() -> true, d -> {
                                    }, drivetrain, gridInterface, intake, manipulator, elevator, elbow)
                                    .andThen(ScoringCommands.stowElevatorCommand(intake, manipulator, elevator, elbow))
                                    .withName("Score"))));
        }

        if (Constants.IS_NT_COMMANDS_ENABLED) {
            elbowPositionChooser.setDefaultOption("High", ElbowPosition.HIGH);
            elbowPositionChooser.addOption("Mid", ElbowPosition.MID);
            elbowPositionChooser.addOption("Low", ElbowPosition.LOW);

            elevatorPositionChooser.setDefaultOption("Top", ElevatorPosition.TOP);
            elevatorPositionChooser.addOption("Cone High", ElevatorPosition.CONE_HIGH);
            elevatorPositionChooser.addOption("Cone Mid", ElevatorPosition.CONE_MID);
            elevatorPositionChooser.addOption("Cone Low", ElevatorPosition.CONE_LOW);
            elevatorPositionChooser.addOption("Cube High", ElevatorPosition.CUBE_HIGH);
            elevatorPositionChooser.addOption("Cube Mid", ElevatorPosition.CUBE_MID);
            elevatorPositionChooser.addOption("Cube Low", ElevatorPosition.CUBE_LOW);
            elevatorPositionChooser.addOption("Bottom", ElevatorPosition.BOTTOM);

            ledStateChooser.setDefaultOption("Rainbow", LEDState.Rainbow);
            ledStateChooser.addOption("TechHOUNDS", LEDState.TechHOUNDS);
            ledStateChooser.addOption("Cone Pickup", LEDState.ConePickup);
            ledStateChooser.addOption("Cone Pickup Flashing", LEDState.ConePickupFlashing);
            ledStateChooser.addOption("CubePickup", LEDState.CubePickup);
            ledStateChooser.addOption("Cube Pickup Flashing", LEDState.CubePickupFlashing);
            ledStateChooser.addOption("Error", LEDState.Error);
            ledStateChooser.addOption("Temporary Error", LEDState.TemporaryError);
            ledStateChooser.addOption("Uninitialized", LEDState.TemporaryError);

            gamePieceChooser.setDefaultOption("Cone", GamePiece.CONE);
            gamePieceChooser.addOption("Cube", GamePiece.CUBE);

            levelChooser.setDefaultOption("High", Level.HIGH);
            levelChooser.addOption("Middle", Level.MIDDLE);
            levelChooser.addOption("Low", Level.LOW);

            robotStateChooser.setDefaultOption("Seeking", RobotState.SEEKING);
            robotStateChooser.addOption("Scoring", RobotState.SCORING);

            LoggingManager.getInstance().addGroup(new LogGroup(
                    new SendableLogger("Commands/Sequences/Choosers", "Game Piece Chooser", gamePieceChooser),
                    new SendableLogger("Commands/Sequences/Choosers", "Level Chooser", levelChooser),
                    new SendableLogger("Commands/Sequences/Scoring", "Raise Elevator",
                            ScoringCommands.raiseElevatorCommand(gamePieceChooser::getSelected,
                                    levelChooser::getSelected, intake, manipulator, elevator, elbow)),
                    new SendableLogger("Commands/Sequences/Scoring", "Place Piece",
                            ScoringCommands.placePieceCommand(() -> secondaryButton,
                                    (s) -> {
                                    }, gamePieceChooser::getSelected, levelChooser::getSelected, drivetrain, intake,
                                    manipulator, elevator, elbow)),
                    new SendableLogger("Commands/Sequences/Scoring", "Place Piece Auto",
                            ScoringCommands.placePieceAutoCommand(gamePieceChooser::getSelected,
                                    levelChooser::getSelected, drivetrain, intake, manipulator, elevator, elbow)),
                    new SendableLogger("Commands/Sequences/Scoring", "Full Score Sequence",
                            ScoringCommands.fullScoreSequenceCommand(() -> secondaryButton,
                                    (s) -> {
                                    }, drivetrain, gridInterface, intake, manipulator, elevator, elbow)),
                    new SendableLogger("Commands/Sequences/Scoring", "Full Score Sequence Auto",
                            ScoringCommands.fullScoreSequenceAutoCommand(gamePieceChooser::getSelected,
                                    levelChooser::getSelected, drivetrain, intake, manipulator, elevator, elbow)),
                    new SendableLogger("Commands/Sequences/Scoring", "Trigger Secondary Button",
                            Commands.sequence(
                                    Commands.runOnce(() -> {
                                        secondaryButton = true;
                                    }),
                                    Commands.waitSeconds(1),
                                    Commands.runOnce(() -> {
                                        secondaryButton = false;
                                    })).withName("Trigger Secondary Button")),
                    new SendableLogger("Commands/Sequences/Scoring", "Stow Elevator",
                            ScoringCommands.stowElevatorCommand(intake, manipulator, elevator, elbow)),
                    new SendableLogger("Commands/Sequences/Intaking", "Start and Run Intaking",
                            IntakingCommands.startAndRunIntakingCommand(gamePieceChooser::getSelected, intake,
                                    manipulator, elevator, elbow)),
                    new SendableLogger("Commands/Sequences/Intaking", "Finish Intaking",
                            IntakingCommands.finishIntakingCommand(gamePieceChooser::getSelected, intake,
                                    manipulator, elevator, elbow)),
                    new SendableLogger("Commands/Sequences/Intaking", "Intake Game Piece",
                            IntakingCommands.intakePieceCommand(() -> secondaryButton, intake,
                                    manipulator, elevator, elbow)),
                    new SendableLogger("Commands/Sequences/Intaking", "Start and Run Ejecting",
                            IntakingCommands.startAndRunEjectingCommand(gamePieceChooser::getSelected, intake,
                                    manipulator, elevator, elbow)),
                    new SendableLogger("Commands/Sequences/Intaking", "Finish Ejecting",
                            IntakingCommands.finishEjectingCommand(gamePieceChooser::getSelected, intake,
                                    manipulator, elevator, elbow)),
                    new SendableLogger("Commands/Sequences/Intaking", "Eject Game Piece",
                            IntakingCommands.ejectPieceCommand(() -> secondaryButton, intake,
                                    manipulator, elevator, elbow)),
                    new SendableLogger("Commands/Sequences/Intaking", "Human Player Pickup",
                            IntakingCommands.humanPlayerPickupCommand(() -> secondaryButton,
                                    (s) -> {
                                    }, gamePieceChooser::getSelected, intake, manipulator, elevator, elbow)),
                    new SendableLogger("Commands/Sequences/Intaking", "Human Player Stow Elevator",
                            IntakingCommands.humanPlayerStowElevatorCommand(intake, manipulator, elevator, elbow)),
                    new SendableLogger("Commands/States", "Robot State Chooser", robotStateChooser),
                    new SendableLogger("Commands/States", "Set Robot State", Modes.setRobotStateCommand(null)),
                    new SendableLogger("Commands/States", "Set Intake Mode",
                            Modes.setIntakeModeCommand(gamePieceChooser::getSelected)),
                    new SendableLogger("Commands/States", "Clear Intake Mode",
                            Modes.clearIntakeModeCommand()),
                    new SendableLogger("Commands/States", "Initialize Mechanisms",
                            Modes.initializeMechanisms(intake, manipulator, elevator, elbow))));

            LoggingManager.getInstance().addGroup(new LogGroup(
                    new SendableLogger("Commands/Manipulator", "Wrist Down",
                            manipulator.setWristDownCommand()),
                    new SendableLogger("Commands/Manipulator", "Wrist Up",
                            manipulator.setWristUpCommand()),
                    new SendableLogger("Commands/Manipulator", "Pincers Open",
                            manipulator.setPincersOpenCommand()),
                    new SendableLogger("Commands/Manipulator", "Pincers Closed",
                            manipulator.setPincersClosedCommand()),
                    new SendableLogger("Commands/Manipulator", "Simulate Pole Switch Triggered",
                            manipulator.simulatePoleSwitchTriggered()),

                    new SendableLogger("Commands/Intake", "Passover Extended",
                            intake.setPassoversExtendedCommand(elevator)),
                    new SendableLogger("Commands/Intake", "Passover Retracted",
                            intake.setPassoversRetractedCommand(elevator)),
                    new SendableLogger("Commands/Intake", "Cubapult Primed",
                            intake.setCubapultReleased()),
                    new SendableLogger("Commands/Intake", "Cubapult Released",
                            intake.setCubapultPrimed()),
                    new SendableLogger("Commands/Intake", "Run Passover Motors",
                            intake.runPassoverMotorsCommand()),
                    new SendableLogger("Commands/Intake", "Simulate Game Piece Detected",
                            intake.simulateGamePieceDetectedCommand()),

                    new SendableLogger("Commands/Elbow", "Setpoint Chooser", elbowPositionChooser),
                    new SendableLogger("Commands/Elbow", "Move to Setpoint",
                            elbow.moveToPositionCommand(elbowPositionChooser::getSelected)
                                    .withName("Move to Elbow Setpoint")),

                    new SendableLogger("Commands/Elevator", "Setpoint Chooser", elevatorPositionChooser),
                    new SendableLogger("Commands/Elevator", "Move to Setpoint",
                            elevator.moveToPositionCommand(elevatorPositionChooser::getSelected)
                                    .withName("Move to Elevator Setpoint")),

                    new SendableLogger("Commands/Drivetrain", "Rotate to 0°",
                            drivetrain.controlledRotateCommand(0, true).withName("Rotate to 0°")),
                    new SendableLogger("Commands/Drivetrain", "Rotate to 90°",
                            drivetrain.controlledRotateCommand(Math.PI / 2, true).withName("Rotate to 90°")),
                    new SendableLogger("Commands/Drivetrain", "Rotate to 180°",
                            drivetrain.controlledRotateCommand(Math.PI, true).withName("Rotate to 180°")),
                    new SendableLogger("Commands/Drivetrain", "Rotate to 270°",
                            drivetrain.controlledRotateCommand(3 * Math.PI / 2, true).withName("Rotate to 270°")),
                    new SendableLogger("Commands/Drivetrain", "Brake O", drivetrain.brakeOCommand()),
                    new SendableLogger("Commands/Drivetrain", "Brake X", drivetrain.brakeXCommand()),

                    new SendableLogger("Commands/LEDs", "LED State Chooser",
                            ledStateChooser),
                    new SendableLogger("Commands/LEDs", "Hold Selected State",
                            leds.holdLEDStateCommand(ledStateChooser::getSelected).withName("Hold Selected LED State")
                                    .ignoringDisable(true))));
        }
    }

}
