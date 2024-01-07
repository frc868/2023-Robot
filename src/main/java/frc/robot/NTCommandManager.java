package frc.robot;

import com.techhounds.houndutil.houndlib.subsystems.BaseSwerveDrive.DriveMode;
import com.techhounds.houndutil.houndlog.LogGroup;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.loggers.SendableLogger;

import frc.robot.Modes.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Elbow.ElbowPosition;
import frc.robot.Constants.Elevator.ElevatorPosition;
import frc.robot.GamePieceLocation.GamePiece;
import frc.robot.GamePieceLocation.Grid;
import frc.robot.GamePieceLocation.GridPosition;
import frc.robot.GamePieceLocation.Level;
import frc.robot.commands.IntakingCommands;
import frc.robot.commands.ScoringCommands;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Elevator;
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
            LoggingManager.getInstance().addGroup("commands/operatorPanel", new LogGroup(
                    new SendableLogger("selectLeftGrid",
                            gridInterface.setGridCommand(Grid.LEFT).withName("Left Grid")),
                    new SendableLogger("selectMiddleGrid",
                            gridInterface.setGridCommand(Grid.MIDDLE).withName("Middle Grid")),
                    new SendableLogger("selectRightGrid",
                            gridInterface.setGridCommand(Grid.RIGHT).withName("Right Grid")),
                    new SendableLogger("selectLeftHighLocation",
                            gridInterface.setLocationCommand(GamePiece.CONE, GridPosition.LEFT, Level.HIGH)
                                    .withName("Left High Location")),
                    new SendableLogger("selectMiddleHighLocation",
                            gridInterface.setLocationCommand(GamePiece.CUBE, GridPosition.MIDDLE, Level.HIGH)
                                    .withName("Middle High Location")),
                    new SendableLogger("selectRightHighLocation",
                            gridInterface.setLocationCommand(GamePiece.CONE, GridPosition.RIGHT, Level.HIGH)
                                    .withName("Right High Location")),
                    new SendableLogger("selectLeftMiddleLocation",
                            gridInterface.setLocationCommand(GamePiece.CONE, GridPosition.LEFT, Level.MIDDLE)
                                    .withName("Left Middle Location")),
                    new SendableLogger("selectMiddleMiddleLocation",
                            gridInterface.setLocationCommand(GamePiece.CUBE, GridPosition.MIDDLE, Level.MIDDLE)
                                    .withName("Middle Middle Location")),
                    new SendableLogger("selectRightMiddleLocation",
                            gridInterface.setLocationCommand(GamePiece.CONE, GridPosition.RIGHT, Level.MIDDLE)
                                    .withName("Right Middle Location")),
                    new SendableLogger("selectLeftLowLocation",
                            gridInterface.setLocationCommand(GamePiece.HYBRID, GridPosition.LEFT, Level.HYBRID)
                                    .withName("Left Low Location")),
                    new SendableLogger("selectMiddleLowLocation",
                            gridInterface.setLocationCommand(GamePiece.HYBRID, GridPosition.MIDDLE, Level.HYBRID)
                                    .withName("Middle Low Location")),
                    new SendableLogger("selectRightLowLocation",
                            gridInterface.setLocationCommand(GamePiece.HYBRID, GridPosition.RIGHT, Level.HYBRID)
                                    .withName("Right Low Location")),
                    new SendableLogger("reset",
                            Commands.runOnce(() -> gridInterface.reset()).withName("Reset")),
                    new SendableLogger("score",
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
            levelChooser.addOption("Low", Level.HYBRID);

            robotStateChooser.setDefaultOption("Seeking", RobotState.SEEKING);
            robotStateChooser.addOption("Scoring", RobotState.SCORING);

            LoggingManager.getInstance().addGroup(new LogGroup(
                    new SendableLogger("commands/sequences/choosers", "gamePieceChooser", gamePieceChooser),
                    new SendableLogger("commands/sequences/choosers", "levelChooser", levelChooser),
                    new SendableLogger("commands/sequences/scoring", "raiseElevator",
                            ScoringCommands.raiseElevatorCommand(gamePieceChooser::getSelected,
                                    levelChooser::getSelected, intake, manipulator, elevator, elbow)),
                    new SendableLogger("commands/sequences/scoring", "placePiece",
                            ScoringCommands.placePieceCommand(() -> secondaryButton,
                                    (s) -> {
                                    }, gamePieceChooser::getSelected, levelChooser::getSelected, drivetrain, intake,
                                    manipulator, elevator, elbow)),
                    new SendableLogger("commands/sequences/scoring", "placePieceAuto",
                            ScoringCommands.placePieceAutoCommand(gamePieceChooser::getSelected,
                                    levelChooser::getSelected, drivetrain, intake, manipulator, elevator, elbow)),
                    new SendableLogger("commands/sequences/scoring", "fullScoreSequence",
                            ScoringCommands.fullScoreSequenceCommand(() -> secondaryButton,
                                    (s) -> {
                                    }, drivetrain, gridInterface, intake, manipulator, elevator, elbow)),
                    new SendableLogger("commands/sequences/scoring", "fullScoreSequenceAuto",
                            ScoringCommands.fullScoreSequenceAutoCommand(gamePieceChooser::getSelected,
                                    levelChooser::getSelected, drivetrain, intake, manipulator, elevator, elbow)),
                    new SendableLogger("commands/sequences/scoring", "triggerSecondaryButton",
                            Commands.sequence(
                                    Commands.runOnce(() -> {
                                        secondaryButton = true;
                                    }),
                                    Commands.waitSeconds(1),
                                    Commands.runOnce(() -> {
                                        secondaryButton = false;
                                    })).withName("Trigger Secondary Button")),
                    new SendableLogger("commands/sequences/scoring", "stowElevator",
                            ScoringCommands.stowElevatorCommand(intake, manipulator, elevator, elbow)),
                    new SendableLogger("commands/sequences/intaking", "startAndRunIntaking",
                            IntakingCommands.startAndRunIntakingCommand(gamePieceChooser::getSelected, intake,
                                    manipulator, elevator, elbow)),
                    new SendableLogger("commands/sequences/intaking", "finishIntaking",
                            IntakingCommands.finishIntakingCommand(gamePieceChooser::getSelected, intake,
                                    manipulator, elevator, elbow)),
                    new SendableLogger("commands/sequences/intaking", "intakeGamePiece",
                            IntakingCommands.intakePieceCommand(() -> secondaryButton, intake,
                                    manipulator, elevator, elbow)),
                    new SendableLogger("commands/sequences/intaking", "startAndRunEjecting",
                            IntakingCommands.startAndRunEjectingCommand(gamePieceChooser::getSelected, intake,
                                    manipulator, elevator, elbow)),
                    new SendableLogger("commands/sequences/intaking", "finishEjecting",
                            IntakingCommands.finishEjectingCommand(gamePieceChooser::getSelected, intake,
                                    manipulator, elevator, elbow)),
                    new SendableLogger("commands/sequences/intaking", "ejectGamePiece",
                            IntakingCommands.ejectPieceCommand(() -> secondaryButton, intake,
                                    manipulator, elevator, elbow)),
                    new SendableLogger("commands/sequences/intaking", "humanPlayerPickup",
                            IntakingCommands.humanPlayerPickupCommand(() -> secondaryButton,
                                    (s) -> {
                                    }, gamePieceChooser::getSelected, intake, manipulator, elevator, elbow)),
                    new SendableLogger("commands/sequences/intaking", "humanPlayerStowElevator",
                            IntakingCommands.humanPlayerStowElevatorCommand(intake, manipulator, elevator, elbow)),
                    new SendableLogger("commands/states", "robotStateChooser", robotStateChooser),
                    new SendableLogger("commands/states", "setRobotState", Modes.setRobotStateCommand(null)),
                    new SendableLogger("commands/states", "setIntakeMode",
                            Modes.setIntakeModeCommand(gamePieceChooser::getSelected)),
                    new SendableLogger("commands/states", "clearIntakeMode",
                            Modes.clearIntakeModeCommand()),
                    new SendableLogger("commands/states", "initializeMechanisms",
                            Modes.initializeMechanisms(intake, manipulator, elevator, elbow))));

            LoggingManager.getInstance().addGroup(new LogGroup(
                    new SendableLogger("commands/manipulator", "wristDown",
                            manipulator.setWristDownCommand()),
                    new SendableLogger("commands/manipulator", "wristUp",
                            manipulator.setWristUpCommand()),
                    new SendableLogger("commands/manipulator", "pincersOpen",
                            manipulator.setPincersOpenCommand()),
                    new SendableLogger("commands/manipulator", "pincersClosed",
                            manipulator.setPincersClosedCommand()),
                    new SendableLogger("commands/manipulator", "simulatePoleSwitchTriggered",
                            manipulator.simulatePoleSwitchTriggered()),

                    new SendableLogger("commands/intake", "passoverExtended",
                            intake.setPassoversExtendedCommand(elevator)),
                    new SendableLogger("commands/intake", "passoverRetracted",
                            intake.setPassoversRetractedCommand(elevator)),
                    new SendableLogger("commands/intake", "cubapultPrimed",
                            intake.setCubapultReleased()),
                    new SendableLogger("commands/intake", "cubapultReleased",
                            intake.setCubapultPrimed()),
                    new SendableLogger("commands/intake", "runPassoverMotors",
                            intake.runPassoverMotorsCommand()),

                    new SendableLogger("commands/elbow", "setpointChooser", elbowPositionChooser),
                    new SendableLogger("commands/elbow", "moveToSetpoint",
                            elbow.moveToPositionCommand(elbowPositionChooser::getSelected)
                                    .withName("Move to Elbow Setpoint")),

                    new SendableLogger("commands/elevator", "setpointChooser", elevatorPositionChooser),
                    new SendableLogger("commands/elevator", "moveToSetpoint",
                            elevator.moveToPositionCommand(elevatorPositionChooser::getSelected)
                                    .withName("Move to Elevator Setpoint")),

                    new SendableLogger("commands/drivetrain", "rotateTo0Degrees",
                            drivetrain.controlledRotateCommand(0, DriveMode.FIELD_ORIENTED).withName("Rotate to 0°")),
                    new SendableLogger("commands/drivetrain", "rotateTo90Degrees",
                            drivetrain.controlledRotateCommand(Math.PI / 2, DriveMode.FIELD_ORIENTED)
                                    .withName("Rotate to 90°")),
                    new SendableLogger("commands/drivetrain", "rotateTo180Degrees",
                            drivetrain.controlledRotateCommand(Math.PI, DriveMode.FIELD_ORIENTED)
                                    .withName("Rotate to 180°")),
                    new SendableLogger("commands/drivetrain", "rotateTo270Degrees",
                            drivetrain.controlledRotateCommand(3 * Math.PI / 2, DriveMode.FIELD_ORIENTED)
                                    .withName("Rotate to 270°")),
                    new SendableLogger("commands/drivetrain", "wheelLock", drivetrain.wheelLockCommand()),

                    new SendableLogger("commands/leds", "ledStateChooser",
                            ledStateChooser),
                    new SendableLogger("commands/leds", "holdSelectedState",
                            leds.holdLEDStateCommand(ledStateChooser::getSelected).withName("Hold Selected LED State")
                                    .ignoringDisable(true))));

        }
    }

}
