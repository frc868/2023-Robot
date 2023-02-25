package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;
import com.techhounds.houndutil.houndauto.AutoManager;
import com.techhounds.houndutil.houndauto.AutoRoutine;
import com.techhounds.houndutil.houndauto.trajectoryloader.TrajectoryLoader;
import com.techhounds.houndutil.houndauto.trajectoryloader.TrajectorySettings;
import com.techhounds.houndutil.houndlog.LogGroup;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.enums.LogLevel;
import com.techhounds.houndutil.houndlog.loggers.SendableLogger;
import com.techhounds.houndutil.houndlog.logitems.BooleanLogItem;
import com.techhounds.houndutil.houndlog.logitems.StringLogItem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.GamePieceLocation.GamePiece;
import frc.robot.GamePieceLocation.Grid;
import frc.robot.GamePieceLocation.GridPosition;
import frc.robot.GamePieceLocation.Level;
import frc.robot.commands.Autos;
import frc.robot.commands.RobotStates;
import frc.robot.commands.Scoring;
import frc.robot.commands.RobotStates.RobotState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Elbow.ElbowPosition;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorPosition;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LEDs.LEDState;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Misc;

/**
 * The container for the robot. Contains subsystems, OI devices, and commands.
 */
public class RobotContainer {
    private Mechanism2d mechanisms = new Mechanism2d(5, 2);
    private MechanismRoot2d root = mechanisms.getRoot("root", 2.5, 0.25);

    private MechanismLigament2d fromRobot = root
            .append(new MechanismLigament2d("fromRobot", -0.33, 0, 0, new Color8Bit(Color.kBlue)));
    private MechanismLigament2d elevatorBaseLigament = fromRobot
            .append(new MechanismLigament2d("elevatorBase", 0.71, 34, 4, new Color8Bit(Color.kCyan)));
    private MechanismLigament2d elevatorLigament = elevatorBaseLigament
            .append(new MechanismLigament2d("elevator", 1.32, 0, 5, new Color8Bit(Color.kOrange)));
    private MechanismLigament2d elbowLigament = elevatorLigament
            .append(new MechanismLigament2d("elbow", 0.2, -34, 3, new Color8Bit(Color.kGreen)));
    private MechanismLigament2d wristLigament = elbowLigament
            .append(new MechanismLigament2d("wrist", 0.2, 90, 3, new Color8Bit(Color.kRed)));

    private MechanismLigament2d toIntake = root
            .append(new MechanismLigament2d("toIntake", 0.33, 0, 0, new Color8Bit(Color.kBlue)));
    private MechanismLigament2d intakeLigament = toIntake
            .append(new MechanismLigament2d("intake", 0.6, 0, 5, new Color8Bit(Color.kPurple)));

    // private final Watchtower watchtower = new Watchtower();
    private final Drivetrain drivetrain = new Drivetrain();
    private final Elbow elbow = new Elbow(elbowLigament);
    private final Elevator elevator = new Elevator(elevatorLigament);
    private final Intake intake = new Intake(intakeLigament);
    private final Manipulator manipulator = new Manipulator(wristLigament);
    private final LEDs leds = new LEDs();
    private final Misc misc = new Misc();

    private final GridInterface gridInterface = new GridInterface();

    private final SendableChooser<ElevatorPosition> elevatorPositionChooser = new SendableChooser<ElevatorPosition>();
    private final SendableChooser<ElbowPosition> elbowPositionChooser = new SendableChooser<ElbowPosition>();

    /**
     * Constructs the robot container.
     */
    public RobotContainer() {
        // watchtower.setPoseEstimator(drivetrain.getPoseEstimator());
        SmartDashboard.putData("Mechanisms", mechanisms);
        elbowLigament.setAngle(-42 - 20);
        DataLogManager.logNetworkTables(true);
        DataLogManager.start();

        LiveWindow.disableAllTelemetry(); // livewindow is basically deprecated. using houndlog instead.
        if (RobotBase.isSimulation()) {
            // prevents annoying joystick disconnected warning
            DriverStation.silenceJoystickConnectionWarning(true);
        }
        configureButtonBindings();
        configureAuto();
        configureNTCommands();

    }

    private void configureButtonBindings() {
        Controls.configureDriverControls(0, drivetrain, gridInterface, intake, manipulator,
                elevator, elbow, leds);
        Controls.configureOperatorControls(1, 2, drivetrain, gridInterface, intake, manipulator,
                elevator, elbow, leds);
        Controls.configureBackupOperatorControls(3, gridInterface, intake, manipulator, elevator,
                elbow, leds);
        // Controls.configureOverridesControls(4, 5, drivetrain, intake, manipulator,
        // elevator, elbow, leds);
    }

    private void configureAuto() {
        TrajectoryLoader.addSettings(
                new TrajectorySettings("Circle").withMaxVelocity(0.5).withMaxAcceleration(1),
                new TrajectorySettings("Figure8").withMaxVelocity(1).withMaxAcceleration(3),
                new TrajectorySettings("1 Piece Hold N").withMaxVelocity(3).withMaxAcceleration(4),
                new TrajectorySettings("2 Piece N").withMaxVelocity(4.2).withMaxAcceleration(8),
                new TrajectorySettings("3 Piece N").withMaxVelocity(4.2).withMaxAcceleration(8),
                new TrajectorySettings("Charge Station N").withMaxVelocity(3).withMaxAcceleration(2));
        TrajectoryLoader.loadAutoPaths();

        AutoManager.getInstance().addEvent("intakeGamePiece",
                RobotStates.intakeGamePieceAutoCommand(intake, manipulator, elevator, elbow, leds));

        AutoManager.getInstance().addRoutine(
                new AutoRoutine("Charge Station N",
                        Autos.pathPlannerTrajectory(TrajectoryLoader.getAutoPath("Charge Station N"), drivetrain)));

        AutoManager.getInstance().addRoutine(
                new AutoRoutine("2 Piece N",
                        Autos.twoPieceN(TrajectoryLoader.getAutoPath("2 Piece N"), drivetrain, intake, manipulator,
                                elevator, elbow, leds)));
        // FieldConstants.displayAutoDriveOnField();
    }

    private void configureNTCommands() {
        if (Constants.IS_VIRTUAL_BUTTON_PANEL_ENABLED) {
            LoggingManager.getInstance().addGroup("Operator Panel", new LogGroup(
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
                    new SendableLogger("Select Intake Mode Cone",
                            RobotStates.setIntakeModeCommand(GamePiece.CONE, leds).withName("Intake Mode Cone")
                                    .repeatedly()),
                    new SendableLogger("Select Intake Mode Cube",
                            RobotStates.setIntakeModeCommand(GamePiece.CUBE, leds).withName("Intake Mode Cube")
                                    .repeatedly()),
                    new SendableLogger("Reset",
                            Commands.runOnce(() -> gridInterface.reset()).withName("Reset")),
                    new SendableLogger("Score",
                            Scoring
                                    .scoreGamePieceCommand(true, () -> true, d -> {
                                    }, drivetrain, gridInterface, intake, manipulator, elevator, elbow)
                                    .andThen(RobotStates.stowElevatorCommand(intake, manipulator, elevator, elbow))
                                    .withName("Score"))));
        }

        if (Constants.IS_NT_COMMANDS_ENABLED) {
            elbowPositionChooser.addOption("High", ElbowPosition.HIGH);
            elbowPositionChooser.addOption("Mid", ElbowPosition.MID);
            elbowPositionChooser.addOption("Low", ElbowPosition.LOW);

            elevatorPositionChooser.addOption("Top", ElevatorPosition.TOP);
            elevatorPositionChooser.addOption("Cone High", ElevatorPosition.CONE_HIGH);
            elevatorPositionChooser.addOption("Cone Mid", ElevatorPosition.CONE_MID);
            elevatorPositionChooser.addOption("Cone Low", ElevatorPosition.CONE_LOW);
            elevatorPositionChooser.addOption("Cube High", ElevatorPosition.CUBE_HIGH);
            elevatorPositionChooser.addOption("Cube Mid", ElevatorPosition.CUBE_MID);
            elevatorPositionChooser.addOption("Cube Low", ElevatorPosition.CUBE_LOW);
            elevatorPositionChooser.addOption("Bottom", ElevatorPosition.BOTTOM);

            LoggingManager.getInstance().addGroup("Main", new LogGroup(
                    new StringLogItem("Intake Mode",
                            () -> RobotStates.getIntakeMode().isEmpty() ? "null"
                                    : RobotStates.getIntakeMode().get().toString(),
                            LogLevel.MAIN),
                    new StringLogItem("RobotState Mode",
                            () -> RobotStates.getCurrentState().toString(),
                            LogLevel.MAIN),
                    new SendableLogger("Drive to Scoring Location",
                            RobotStates.autoDriveCommand(drivetrain, gridInterface)),
                    new SendableLogger("Drive to Scoring Location2",
                            RobotStates.autoDriveCommand(() -> RobotState.SCORING, () -> GamePieceLocation.A1,
                                    drivetrain)),
                    new SendableLogger("Set Seeking State",
                            RobotStates.setCurrentStateCommand(RobotState.SEEKING)),
                    new SendableLogger("Set Scoring State",
                            RobotStates.setCurrentStateCommand(RobotState.SCORING)),
                    new SendableLogger("Full Auto Score with movement",
                            Autos.fullAutoScoreWithMovement(GamePieceLocation.A1, drivetrain, intake, manipulator,
                                    elevator, elbow).withName("Full Auto Score")),
                    new SendableLogger("Full Auto Intake with movement",
                            Autos.fullAutoIntakeWithMovement(GamePiece.CONE, drivetrain, intake, manipulator,
                                    elevator, elbow, leds).withName("Full Auto Intake")),
                    new SendableLogger("EVERYTHING",
                            Commands.sequence(
                                    Commands.print("1"),
                                    RobotStates.autoDriveCommand(drivetrain, gridInterface),
                                    Commands.print("2"),
                                    RobotStates.intakeGamePieceAutoCommand(intake, manipulator, elevator, elbow,
                                            leds),
                                    Commands.print("3"),
                                    RobotStates.autoDriveCommand(drivetrain, gridInterface),
                                    Commands.print("4"),
                                    Scoring.scoreGamePieceCommand(true, () -> true, d -> {
                                    }, drivetrain, gridInterface, intake, manipulator,
                                            elevator,
                                            elbow),
                                    Commands.print("5"),
                                    RobotStates.stowElevatorCommand(intake, manipulator, elevator, elbow),
                                    Commands.print("6"))
                                    .withName("Sicko Mode").repeatedly()),
                    new SendableLogger("Initialize Mechanisms",
                            RobotStates.initializeMechanisms(intake, manipulator, elevator, elbow,
                                    leds)),
                    new SendableLogger("Intake Game Piece",
                            RobotStates.intakeGamePiece(true, true, () -> true, intake, manipulator,
                                    elevator, elbow,
                                    leds)),
                    new SendableLogger("Score Game Piece",
                            Scoring.scoreGamePieceCommand(true, () -> true, d -> {
                            },
                                    drivetrain, gridInterface, intake, manipulator, elevator, elbow)),
                    new SendableLogger("Score Game Piece w Stow",
                            Scoring.scoreGamePieceCommand(true, () -> false, d -> {
                            }, drivetrain, gridInterface, intake, manipulator, elevator, elbow)
                                    .andThen(RobotStates.stowElevatorCommand(intake, manipulator, elevator, elbow))
                                    .withName("Score Game Piece w Stow")),
                    new SendableLogger("Auto Score Game Piece w Stow",
                            Scoring
                                    .scoreGamePieceAutoCommand(() -> GamePiece.CONE, () -> Level.HIGH, drivetrain,
                                            intake,
                                            manipulator, elevator, elbow)
                                    .andThen(RobotStates.stowElevatorCommand(intake, manipulator, elevator, elbow))
                                    .withName("Auto Score Game Piece w Stow")),
                    new SendableLogger("Stow Elevator",
                            RobotStates.stowElevatorCommand(intake, manipulator, elevator, elbow)),
                    new SendableLogger("Drivetrain", drivetrain),
                    new SendableLogger("Elbow", elbow),
                    new SendableLogger("Elevator", elevator),
                    new SendableLogger("Intake", intake),
                    new SendableLogger("Manipulator", manipulator),
                    new SendableLogger("LEDs", leds),
                    new SendableLogger("Misc", misc),
                    new StringLogItem("Current Discrete Error",
                            () -> RobotStates.getCurrentDiscreteError().orElse("none"),
                            LogLevel.MAIN),
                    new StringLogItem("Current Continuous Error",
                            () -> RobotStates.getCurrentContinuousError().orElse("none"), LogLevel.MAIN),
                    new BooleanLogItem("Is Initialized", () -> RobotStates.isInitialized(),
                            LogLevel.MAIN),
                    new BooleanLogItem("Intake Mode Bool",
                            () -> RobotStates.getIntakeMode().orElse(GamePiece.HYBRID) == GamePiece.CONE,
                            LogLevel.MAIN)));

            LoggingManager.getInstance().addGroup(new LogGroup(
                    new SendableLogger("Manipulator", "Wrist Down",
                            manipulator.setWristDownCommand()),
                    new SendableLogger("Manipulator", "Wrist Up",
                            manipulator.setWristUpCommand(elevator)),
                    new SendableLogger("Manipulator", "Pincers Open",
                            manipulator.setPincersOpenCommand()),
                    new SendableLogger("Manipulator", "Pincers Closed",
                            manipulator.setPincersClosedCommand()),
                    new SendableLogger("Manipulator", "Sim Pole Switch Triggered",
                            manipulator.simPoleSwitchTriggered()),
                    new SendableLogger("Intake", "Passover Extended",
                            intake.setPassoversExtendedCommand(elevator)),
                    new SendableLogger("Intake", "Passover Retracted",
                            intake.setPassoversRetractedCommand(elevator)),
                    new SendableLogger("Intake", "Intake Up",
                            intake.setIntakeUpCommand(elevator)),
                    new SendableLogger("Intake", "Intake Down",
                            intake.setIntakeDownCommand(elevator)),
                    new SendableLogger("Intake", "Run Passover Motors",
                            intake.runPassoverMotorsCommand()),
                    new SendableLogger("Intake", "Sim Game Piece Detected",
                            intake.simGamePieceDetectedCommand()),
                    new SendableLogger("Elbow", "Set State",
                            elbow.setDesiredPositionCommand(elbowPositionChooser.getSelected(), elevator)
                                    .withName("Set Elbow State")),
                    new SendableLogger("Elbow", "Disable", Commands.runOnce(elbow::disable)),
                    new SendableLogger("Elevator", "Set State",
                            elevator.setDesiredPositionCommand(elevatorPositionChooser.getSelected(),
                                    intake, elbow)
                                    .withName("Set Elevator State")),
                    new SendableLogger("Elevator", "Disable",
                            Commands.runOnce(elevator::disable)),
                    new SendableLogger("Drivetrain", "Turn CCW",
                            drivetrain.turnWhileMovingCommand(true)),
                    new SendableLogger("Drivetrain", "Turn CW",
                            drivetrain.turnWhileMovingCommand(false)),
                    new SendableLogger("Drivetrain", "BrakeO", drivetrain.brakeCommand()),
                    new SendableLogger("Drivetrain", "BrakeX", drivetrain.brakeXCommand()),
                    new SendableLogger("Drivetrain", "Path Follow",
                            Commands.runOnce(
                                    () -> AutoManager.getInstance().getField().getObject("Traj")
                                            .setTrajectory(PathPlanner.generatePath(
                                                    new PathConstraints(2, 3),
                                                    new PathPoint(drivetrain.getPose().getTranslation(),
                                                            drivetrain.getPose().getRotation()),
                                                    new PathPoint(
                                                            drivetrain.getPose()
                                                                    .plus(new Transform2d(new Translation2d(3, 0),
                                                                            new Rotation2d(0)))
                                                                    .getTranslation(),
                                                            Rotation2d.fromDegrees(0)))))
                                    .andThen(
                                            new ProxyCommand(
                                                    () -> drivetrain.pathFollowingCommand(PathPlanner.generatePath(
                                                            new PathConstraints(2, 3),
                                                            new PathPoint(drivetrain.getPose().getTranslation(),
                                                                    drivetrain.getPose().getRotation()),
                                                            new PathPoint(
                                                                    drivetrain.getPose()
                                                                            .plus(new Transform2d(
                                                                                    new Translation2d(3, 0),
                                                                                    new Rotation2d(0)))
                                                                            .getTranslation(),
                                                                    Rotation2d.fromDegrees(0))))))
                                    .andThen(() -> drivetrain.stop())
                                    .withName("Follow Path 3m")),
                    new SendableLogger("LEDs", "Rainbow",
                            leds.setLEDStateCommand(LEDState.Rainbow).withName("LED Rainbow").ignoringDisable(true)),
                    new SendableLogger("LEDs", "TechHOUNDS",
                            leds.setLEDStateCommand(LEDState.TechHOUNDS).withName("LED TechHOUNDS")
                                    .ignoringDisable(true)),
                    new SendableLogger("LEDs", "Cone Pickup",
                            leds.setLEDStateCommand(LEDState.ConePickup).withName("LED Cone Pickup")
                                    .ignoringDisable(true)),
                    new SendableLogger("LEDs", "Cube Pickup",
                            leds.setLEDStateCommand(LEDState.CubePickup).withName("LED Cube Pickup")
                                    .ignoringDisable(true)),
                    new SendableLogger("LEDs", "Error",
                            leds.setLEDStateCommand(LEDState.Error).withName("LED Error").ignoringDisable(true)),
                    new SendableLogger("LEDs", "Uninitialized",
                            leds.setLEDStateCommand(LEDState.Uninitialized).withName("LED Uninitialized")
                                    .ignoringDisable(true)),
                    new SendableLogger("LEDs", "Go To Previous State",
                            leds.setPreviousLEDStateCommand().withName("LED Previous State").ignoringDisable(true))));
        }
    }
}