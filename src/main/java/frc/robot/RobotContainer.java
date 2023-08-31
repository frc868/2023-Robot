package frc.robot;

import java.util.function.Supplier;

import com.techhounds.houndutil.houndauto.AutoManager;
import com.techhounds.houndutil.houndauto.trajectoryloader.TrajectoryLoader;
import com.techhounds.houndutil.houndauto.trajectoryloader.TrajectorySettings;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.interfaces.Log;
import com.techhounds.houndutil.houndlog.interfaces.SendableLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
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
import frc.robot.subsystems.Watchtower;

/**
 * The container for the robot. Contains subsystems, OI devices, and commands.
 */
public class RobotContainer {
    @SendableLog(name = "Mechanisms", groups = "WPILib")
    private Mechanism2d mechanisms = new Mechanism2d(5, 2);
    private MechanismRoot2d root = mechanisms.getRoot("root", 2.5, 0.25);

    private MechanismLigament2d fromRobot = root
            .append(new MechanismLigament2d("fromRobot", -0.33, 0, 0, new Color8Bit(Color.kWhite)));
    private MechanismLigament2d elevatorBaseLigament = fromRobot
            .append(new MechanismLigament2d("elevatorBase", 0.71, 34, 4, new Color8Bit(Color.kCyan)));
    private MechanismLigament2d elevatorLigament = elevatorBaseLigament
            .append(new MechanismLigament2d("elevator", 1.32, 0, 5, new Color8Bit(Color.kOrange)));
    private MechanismLigament2d elbowLigament = elevatorLigament
            .append(new MechanismLigament2d("elbow", 0.2, -34, 3, new Color8Bit(Color.kGreen)));
    private MechanismLigament2d wristLigament = elbowLigament
            .append(new MechanismLigament2d("wrist", 0.2, 90, 3, new Color8Bit(Color.kRed)));

    private MechanismLigament2d toCubapult = fromRobot
            .append(new MechanismLigament2d("toCubapult", 0.3, 90, 1, new Color8Bit(Color.kWhite)));
    private MechanismLigament2d cubapultLigament = toCubapult
            .append(new MechanismLigament2d("cubapult", 0.3, 0, 3, new Color8Bit(Color.kPurple)));

    @SuppressWarnings("unused")
    private MechanismLigament2d cubapultExtLigament = cubapultLigament
            .append(new MechanismLigament2d("cubapultExt", 0.07, 90, 3, new Color8Bit(Color.kPurple)));

    @Log(name = "Watchtower", groups = "Subsystems")
    @SendableLog(name = "Watchtower", groups = { "WPILib", "Subsystem Info" })
    private final Watchtower watchtower = new Watchtower();

    @Log(name = "Drivetrain", groups = "Subsystems")
    @SendableLog(name = "Drivetrain", groups = { "WPILib", "Subsystem Info" })
    private final Drivetrain drivetrain = new Drivetrain();

    @Log(name = "Elbow", groups = "Subsystems")
    @SendableLog(name = "Elbow", groups = { "WPILib", "Subsystem Info" })
    private final Elbow elbow = new Elbow(elbowLigament);

    @Log(name = "Elevator", groups = "Subsystems")
    @SendableLog(name = "Elevator", groups = { "WPILib", "Subsystem Info" })
    private final Elevator elevator = new Elevator(elevatorLigament);

    @Log(name = "Intake", groups = "Subsystems")
    @SendableLog(name = "Intake", groups = { "WPILib", "Subsystem Info" })
    private final Intake intake = new Intake(cubapultLigament);

    @Log(name = "Manipulator", groups = "Subsystems")
    @SendableLog(name = "Manipulator", groups = { "WPILib", "Subsystem Info" })
    private final Manipulator manipulator = new Manipulator(wristLigament);

    @Log(name = "LEDs", groups = "Subsystems")
    @SendableLog(name = "LEDs", groups = { "WPILib", "Subsystem Info" })
    private final LEDs leds = new LEDs();

    private final GridInterface gridInterface = new GridInterface();

    /** The PDH (CAN ID 1) */
    @Log(name = "PDH", groups = { "Subsystems", "Misc" })
    private PowerDistribution pdh = new PowerDistribution();
    /** The PH (CAN ID 1) */
    @Log(name = "Pneumatic Hub", groups = { "Subsystems", "Misc" })
    private PneumaticHub ph = new PneumaticHub();

    @SendableLog(name = "CommandScheduler", groups = "WPILib")
    private CommandScheduler commandScheduler = CommandScheduler.getInstance();

    @Log(name = "Robot State", groups = "Modes")
    private Supplier<String> robotStateSupplier = () -> Modes.getRobotState().toString();

    @Log(name = "Intake Mode", groups = "Modes")
    private Supplier<String> intakeModeSupplier = () -> Modes.getIntakeMode().isPresent()
            ? Modes.getIntakeMode().orElseThrow().toString()
            : "None";

    @Log(name = "Is Initialized", groups = "Modes")
    private Supplier<Boolean> initializedSupplier = Modes::isInitialized;

    @Log(name = "Discrete Error", groups = "Modes")
    private Supplier<String> discreteErrorSupplier = () -> Modes.getCurrentDiscreteError().isPresent()
            ? Modes.getCurrentDiscreteError().orElseThrow().toString()
            : "None";

    @Log(name = "Continuous Error", groups = "Modes")
    private Supplier<String> continuousErrorSupplier = () -> Modes.getCurrentContinuousError().isPresent()
            ? Modes.getCurrentContinuousError().orElseThrow().toString()
            : "None";

    /**
     * Constructs the robot container.
     */
    public RobotContainer() {
        watchtower.setPoseEstimator(drivetrain.getPoseEstimator());
        DataLogManager.logNetworkTables(true);
        DriverStation.startDataLog(DataLogManager.getLog());
        DataLogManager.start();

        LoggingManager.getInstance().registerRobotContainer(this);

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
        Controls.configureOperatorControls(1, 2, drivetrain, gridInterface, intake, manipulator,
                elevator, elbow);
        Controls.configureBackupOperatorControls(3, gridInterface, intake, manipulator, elevator,
                elbow);
        Controls.configureOverridesControls(4, 5, drivetrain, intake, manipulator,
                elevator, elbow);
    }

    private void configureAuto() {
        TrajectoryLoader.addSettings(
                new TrajectorySettings("Full N").withMaxVelocity(4.4).withMaxAcceleration(4),
                new TrajectorySettings("3 Piece Link N").withMaxVelocity(3).withMaxAcceleration(2),
                new TrajectorySettings("2 Piece Hold Charge M").withMaxVelocity(4).withMaxAcceleration(2.5),
                new TrajectorySettings("1 Piece Charge Mobility M").withMaxVelocity(3).withMaxAcceleration(2),
                new TrajectorySettings("1 Piece Charge M").withMaxVelocity(4.4).withMaxAcceleration(2),
                new TrajectorySettings("1 Piece Mobility S").withMaxVelocity(3.5).withMaxAcceleration(2),
                new TrajectorySettings("1 Piece Mobility B").withMaxVelocity(3.5).withMaxAcceleration(2),
                new TrajectorySettings("3 Piece Link S").withMaxVelocity(4.4).withMaxAcceleration(3));
        TrajectoryLoader.loadAutoPaths();

        AutoManager.getInstance().addRoutine(Autos.northOnePiece(drivetrain, intake, manipulator, elevator, elbow));
        AutoManager.getInstance().addRoutine(Autos.southOnePiece(drivetrain, intake, manipulator, elevator, elbow));
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