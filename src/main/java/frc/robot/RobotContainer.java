package frc.robot;

import com.techhounds.houndutil.houndauto.AutoManager;
import com.techhounds.houndutil.houndauto.AutoRoutine;
import com.techhounds.houndutil.houndauto.trajectoryloader.TrajectoryLoader;
import com.techhounds.houndutil.houndauto.trajectoryloader.TrajectorySettings;
import com.techhounds.houndutil.houndlog.LogGroup;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.loggers.SendableLogger;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.commands.Autos;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Misc;
import frc.robot.subsystems.Watchtower;

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

    private final Watchtower watchtower = new Watchtower();
    private final Drivetrain drivetrain = new Drivetrain();
    private final Elbow elbow = new Elbow(elbowLigament);
    private final Elevator elevator = new Elevator(elevatorLigament);
    private final Intake intake = new Intake(intakeLigament);
    private final Manipulator manipulator = new Manipulator(wristLigament);
    private final LEDs leds = new LEDs();
    private final Misc misc = new Misc();

    private final GridInterface gridInterface = new GridInterface();
    private final AutoGenerator autoGenerator = new AutoGenerator(drivetrain,
            intake, manipulator, elevator, elbow,
            leds);

    /**
     * Constructs the robot container.
     */
    public RobotContainer() {
        watchtower.setPoseEstimator(drivetrain.getPoseEstimator());
        DataLogManager.logNetworkTables(true);
        DriverStation.startDataLog(DataLogManager.getLog());
        DataLogManager.start();

        LoggingManager.getInstance().addGroup("Main",
                new LogGroup(
                        new SendableLogger("Mechanisms", mechanisms),
                        new SendableLogger("Subsystems/Drivetrain", drivetrain),
                        new SendableLogger("Subsystems/Elbow", elbow),
                        new SendableLogger("Subsystems/Elevator", elevator),
                        new SendableLogger("Subsystems/Intake", intake),
                        new SendableLogger("Subsystems/Manipulator", manipulator),
                        new SendableLogger("Subsystems/LEDs", leds),
                        new SendableLogger("Subsystems/Misc", misc),
                        new SendableLogger("CommandScheduler", CommandScheduler.getInstance())));

        LiveWindow.disableAllTelemetry(); // livewindow is basically deprecated. using houndlog instead.
        if (RobotBase.isSimulation()) {
            // prevents annoying joystick disconnected warning
            DriverStation.silenceJoystickConnectionWarning(true);
        }
        configureButtonBindings();
        configureAuto();
        SimManager.configureNTCommands(drivetrain, gridInterface, intake, manipulator, elevator, elbow, leds, misc);

    }

    private void configureButtonBindings() {
        Controls.configureDriverControls(0, drivetrain, gridInterface, intake, manipulator,
                elevator, elbow);
        Controls.configureOperatorControls(1, 2, drivetrain, gridInterface, intake, manipulator,
                elevator, elbow);
        Controls.configureBackupOperatorControls(3, gridInterface, intake, manipulator, elevator,
                elbow);
        Controls.configureOverridesControls(4, 5, drivetrain, intake, manipulator,
                elevator, elbow);
    }

    private void configureAuto() {
        TrajectoryLoader.addSettings(
                new TrajectorySettings("2 Piece N").withMaxVelocity(4.4).withMaxAcceleration(2),
                new TrajectorySettings("2 Piece Charge N").withMaxVelocity(4.4).withMaxAcceleration(3.2),
                new TrajectorySettings("2 Piece S").withMaxVelocity(4.4).withMaxAcceleration(2),
                new TrajectorySettings("2 Piece Charge S").withMaxVelocity(4.4).withMaxAcceleration(2),
                new TrajectorySettings("1 Piece Charge M").withMaxVelocity(4.4).withMaxAcceleration(2),
                new TrajectorySettings("2 Piece Charge M").withMaxVelocity(4.4).withMaxAcceleration(2),
                new TrajectorySettings("Charge Station M").withMaxVelocity(4.4).withMaxAcceleration(3));
        TrajectoryLoader.loadAutoPaths();

        // AutoManager.getInstance().addEvent("startIntakingCone",
        // RobotStates.startIntakingCommand(() -> GamePiece.CONE, intake, manipulator,
        // elevator, elbow));
        // AutoManager.getInstance().addEvent("startIntakingCube",
        // RobotStates.startIntakingCommand(() -> GamePiece.CUBE, intake, manipulator,
        // elevator, elbow));
        // AutoManager.getInstance().addEvent("endIntakingCone",
        // RobotStates.endIntakingCommand(() -> GamePiece.CUBE, intake, manipulator,
        // elevator, elbow));
        // AutoManager.getInstance().addEvent("endIntakingCube",
        // RobotStates.endIntakingCommand(() -> GamePiece.CUBE, intake, manipulator,
        // elevator, elbow));

        AutoManager.getInstance().addRoutine(
                new AutoRoutine("2 Piece N",
                        Autos.twoPieceN(TrajectoryLoader.getAutoPath("2 Piece N"), drivetrain, intake, manipulator,
                                elevator, elbow)));
        AutoManager.getInstance().addRoutine(
                new AutoRoutine("2 Piece Charge N",
                        Autos.twoPieceChargeN(TrajectoryLoader.getAutoPath("2 Piece Charge N"), drivetrain, intake,
                                manipulator,
                                elevator, elbow)));
        AutoManager.getInstance().addRoutine(
                new AutoRoutine("3 Piece N",
                        Autos.threePieceN(TrajectoryLoader.getAutoPath("3 Piece N"), drivetrain, intake, manipulator,
                                elevator, elbow)));

        AutoManager.getInstance().addRoutine(
                new AutoRoutine("2 Piece S",
                        Autos.twoPieceS(TrajectoryLoader.getAutoPath("2 Piece S"), drivetrain, intake, manipulator,
                                elevator, elbow)));
        AutoManager.getInstance().addRoutine(
                new AutoRoutine("2 Piece Charge S",
                        Autos.twoPieceChargeS(TrajectoryLoader.getAutoPath("2 Piece Charge S"), drivetrain, intake,
                                manipulator,
                                elevator, elbow)));

        AutoManager.getInstance().addRoutine(
                new AutoRoutine("3 Piece S",
                        Autos.threePieceS(TrajectoryLoader.getAutoPath("3 Piece S"), drivetrain, intake, manipulator,
                                elevator, elbow)));

        AutoManager.getInstance().addRoutine(
                new AutoRoutine("1 Piece Charge M",
                        Autos.onePieceChargeM(
                                TrajectoryLoader.getAutoPath("1 Piece Charge M"),
                                drivetrain, intake, manipulator, elevator, elbow)));
        AutoManager.getInstance().addRoutine(
                new AutoRoutine("2 Piece Charge M",
                        Autos.twoPieceChargeM(
                                TrajectoryLoader.getAutoPath("2 Piece Charge M"),
                                drivetrain, intake, manipulator, elevator, elbow)));
        AutoManager.getInstance().addRoutine(
                new AutoRoutine("1 Piece N",
                        Autos.onePieceN(drivetrain, intake, manipulator, elevator, elbow)));
        AutoManager.getInstance().addRoutine(
                new AutoRoutine("1 Piece S",
                        Autos.onePieceS(drivetrain, intake, manipulator, elevator, elbow)));
        AutoManager.getInstance().addRoutine(
                new AutoRoutine("Do Nothing",
                        Autos.doNothing()));
        AutoManager.getInstance().addRoutine(
                new AutoRoutine("Charge Station M",
                        Autos.chargeM(TrajectoryLoader.getAutoPath("Charge Station M"), drivetrain)));
        AutoManager.getInstance().addRoutine(
                new AutoRoutine("AutoGenerator",
                        autoGenerator.getAutoCommand()));
        // FieldConstants.displayAutoDriveOnField();
        // FieldConstants.displayItemsOnField();
    }
}