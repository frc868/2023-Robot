package frc.robot;

import com.techhounds.houndutil.houndlib.SparkMaxConfigurator;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.interfaces.Log;
import com.techhounds.houndutil.houndlog.interfaces.SendableLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Watchtower;
import static frc.robot.Constants.Mechanisms.*;

import java.util.ArrayList;
import java.util.function.Supplier;

/**
 * The container for the robot. Contains subsystems, OI devices, and commands.
 */
public class RobotContainer {
    @SendableLog(groups = "wpilib")
    public static Mechanism2d mechanisms = new Mechanism2d(5, 2);

    @Log(groups = "subsystems")
    @SendableLog(groups = { "wpilib", "subsystems" })
    private final Drivetrain drivetrain = new Drivetrain();

    @Log(groups = "subsystems")
    @SendableLog(groups = { "wpilib", "subsystems" })
    private final Elevator elevator = new Elevator(ELEVATOR_LIGAMENT);

    @Log(groups = "subsystems")
    @SendableLog(groups = { "wpilib", "subsystems" })
    private final Elbow elbow = new Elbow(elevator::getCarriagePose, ELBOW_LIGAMENT);

    @Log(groups = "subsystems")
    @SendableLog(groups = { "wpilib", "subsystems" })
    private final Intake intake = new Intake(CUBAPULT_LIGAMENT);

    @Log(groups = "subsystems")
    @SendableLog(groups = { "wpilib", "subsystems" })
    private final Manipulator manipulator = new Manipulator(elbow::getComponentPose, WRIST_LIGAMENT);

    @Log(groups = "subsystems")
    @SendableLog(groups = { "wpilib", "subsystems" })
    private final LEDs leds = new LEDs();

    @Log(groups = "subsystems")
    @SendableLog(groups = { "wpilib", "subsystems" })
    private final Watchtower watchtower = new Watchtower();

    private final GridInterface gridInterface = new GridInterface();

    @Log(groups = { "subsystems", "misc" })
    private final PowerDistribution pdh = new PowerDistribution();

    @Log(groups = { "subsystems", "misc" })
    private final PneumaticHub ph = new PneumaticHub();

    @SendableLog(groups = "wpilib")
    private final CommandScheduler commandScheduler = CommandScheduler.getInstance();

    private double prevLoopTime = 0.0;

    @Log(groups = "wpilib")
    private final Supplier<Double> loopTimeMs = () -> {
        double timestamp = Timer.getFPGATimestamp();
        double loopTime = Timer.getFPGATimestamp() - prevLoopTime;
        prevLoopTime = timestamp;
        return loopTime * 1000.0;
    };

    /**
     * Constructs the robot container.
     */
    public RobotContainer() {
        SparkMaxConfigurator.safeBurnFlash();
        watchtower.setPoseEstimator(drivetrain.getPoseEstimator());
        watchtower.setSimPoseSupplier(drivetrain::getSimPose);
        DataLogManager.logNetworkTables(true);
        DriverStation.startDataLog(DataLogManager.getLog());
        DataLogManager.start();

        LoggingManager.getInstance().registerRobotContainer(this);
        LoggingManager.getInstance().registerClass(Modes.class, "modes", new ArrayList<String>());

        LiveWindow.disableAllTelemetry(); // livewindow is basically deprecated. using houndlog instead.

        if (RobotBase.isSimulation()) {
            // prevents annoying joystick disconnected warning
            DriverStation.silenceJoystickConnectionWarning(true);
        }
        configureButtonBindings();
        configureAuto();

        NTCommandManager.configureNTCommands(drivetrain, gridInterface, intake, manipulator, elevator, elbow, leds);
    }

    private void configureButtonBindings() {
        Controls.configureDriverControls(0, drivetrain, gridInterface, intake, manipulator,
                elevator, elbow);
        // Controls.configureOperatorControls(1, 2, drivetrain, gridInterface, intake,
        // manipulator,
        // elevator, elbow);
        Controls.configureTestingControls(2, gridInterface, intake, manipulator, elevator,
                elbow);
        Controls.configureBackupOperatorControls(3, gridInterface, intake, manipulator, elevator,
                elbow);
        // Controls.configureOverridesControls(4, 5, drivetrain, intake, manipulator,
        // elevator, elbow);
    }

    private void configureAuto() {
        // TrajectoryLoader.loadAutoPaths();

        // AutoManager.getInstance().addRoutine(Autos.northOnePiece(drivetrain, intake,
        // manipulator, elevator, elbow));
        // AutoManager.getInstance().addRoutine(Autos.southOnePiece(drivetrain, intake,
        // manipulator, elevator, elbow));
        // AutoManager.getInstance().addRoutine(
        // new AutoRoutine("2 Piece Cube N",
        // () -> Autos.threePieceN(drivetrain, intake, manipulator, elevator, elbow)));

        // AutoManager.getInstance().addRoutine(
        // new AutoRoutine("1 Piece Mobility N",
        // () -> Autos.onePieceMobilityN(drivetrain, intake, manipulator, elevator,
        // elbow)));
        // AutoManager.getInstance().addRoutine(
        // new AutoRoutine("1 Piece Charge Mobility M",
        // () -> Autos.onePieceChargeMobilityM(drivetrain, intake, manipulator,
        // elevator, elbow)));
        // AutoManager.getInstance().addRoutine(
        // new AutoRoutine("1 Piece Charge M",
        // () -> Autos.onePieceChargeM(drivetrain, intake, manipulator, elevator,
        // elbow)));
        // AutoManager.getInstance().addRoutine(
        // new AutoRoutine("1 Piece Mobility S",
        // () -> Autos.onePieceMobilityS(drivetrain, intake, manipulator, elevator,
        // elbow)));

        // FieldConstants.displayAutoDriveOnField();
        // FieldConstants.displayItemsOnField();
    }
}