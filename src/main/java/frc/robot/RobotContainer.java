package frc.robot;

import com.techhounds.houndutil.houndauto.AutoManager;
import com.techhounds.houndutil.houndauto.AutoRoutine;
import com.techhounds.houndutil.houndauto.trajectoryloader.TrajectoryLoader;
import com.techhounds.houndutil.houndauto.trajectoryloader.TrajectorySettings;
import com.techhounds.houndutil.houndlog.LogGroup;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.enums.LogLevel;
import com.techhounds.houndutil.houndlog.loggers.Logger;
import com.techhounds.houndutil.houndlog.loggers.SendableLogger;
import com.techhounds.houndutil.houndlog.logitems.StringLogItem;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.Autos;
import frc.robot.commands.RobotStates;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Misc;

/**
 * The container for the robot. Contains subsystems, OI devices, and commands.
 */
public class RobotContainer {
    private Mechanism2d mechanisms = new Mechanism2d(5, 2);
    private MechanismRoot2d root = mechanisms.getRoot("root", 2.5, 0.1);

    private MechanismLigament2d fromRobot = root
            .append(new MechanismLigament2d("fromRobot", -0.33, 0, 0, new Color8Bit(Color.kBlue)));
    private MechanismLigament2d elevatorBaseLigament = fromRobot
            .append(new MechanismLigament2d("elevatorBase", 0.71, 42, 5, new Color8Bit(Color.kCyan)));
    private MechanismLigament2d elevatorLigament = elevatorBaseLigament
            .append(new MechanismLigament2d("elevator", 1.32, 0, 5, new Color8Bit(Color.kOrange)));
    private MechanismLigament2d elbowLigament = elevatorLigament
            .append(new MechanismLigament2d("elbow", 0.2, -42, 5, new Color8Bit(Color.kGreen)));
    private MechanismLigament2d wristLigament = elbowLigament
            .append(new MechanismLigament2d("wrist", 0.2, 90, 5, new Color8Bit(Color.kRed)));

    private final Drivetrain drivetrain = new Drivetrain();
    private final Elbow elbow = new Elbow(elbowLigament);
    private final Elevator elevator = new Elevator(elevatorLigament);
    private final Intake intake = new Intake();
    private final Manipulator manipulator = new Manipulator(wristLigament);
    private final LEDs leds = new LEDs();
    private final GridInterface gridInterface = new GridInterface();
    @SuppressWarnings("unused")
    private final Misc misc = new Misc();

    /**
     * Constructs the robot container.
     */
    public RobotContainer() {
        SmartDashboard.putData("Mechanisms", mechanisms);
        // elevatorMech.setLength(-0.5);
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

        LoggingManager.getInstance().addGroup("Main", new LogGroup(new Logger[] {
                new StringLogItem("Intake Mode",
                        () -> RobotStates.getIntakeMode().isEmpty() ? "null"
                                : RobotStates.getIntakeMode().get().toString(),
                        LogLevel.MAIN),
                new SendableLogger("Prepare to Intake Game Piece",
                        RobotStates.prepareToIntakeGamePiece(intake, manipulator, elevator, elbow, leds)),
                new SendableLogger("Intake Game Piece",
                        RobotStates.intakeGamePiece(intake, manipulator, elevator, elbow, leds)),
                new SendableLogger("Score Game Piece",
                        RobotStates.scoreGamePiece(gridInterface, intake, manipulator, elevator, elbow, leds)),
                new SendableLogger("Stow Elevator",
                        RobotStates.stowElevator(intake, manipulator, elevator, elbow, leds)),
                new SendableLogger("Drive to Scoring Location",
                        RobotStates.driveToScoringLocation(drivetrain, gridInterface, leds)) }));

    }

    private void configureButtonBindings() {
        Controls.configureDriverControls(0, drivetrain, intake, manipulator, elevator, elbow, leds);
        Controls.configureOperatorControls(1, 2, gridInterface, intake, manipulator, elevator, elbow, leds);
        Controls.configureBackupOperatorControls(3, intake, manipulator, elevator, elbow, leds);
        Controls.configureOverridesControls(5, 6, drivetrain, intake, manipulator, elevator, elbow, leds);
    }

    public void configureAuto() {
        TrajectoryLoader.addSettings(
                new TrajectorySettings("Circle").withMaxVelocity(0.5).withMaxAcceleration(3),
                new TrajectorySettings("Figure8").withMaxVelocity(1).withMaxAcceleration(3));
        TrajectoryLoader.loadAutoPaths();

        AutoManager.getInstance().addEvent("event1", Commands.print("1"));
        AutoManager.getInstance().addEvent("event2", Commands.print("2"));
        AutoManager.getInstance().addEvent("event3", Commands.print("3"));
        AutoManager.getInstance().addEvent("event4", Commands.print("4"));

        AutoManager.getInstance().addRoutine(
                new AutoRoutine("Circle", Autos.circle(TrajectoryLoader.getAutoPath("Circle"), drivetrain)));
        AutoManager.getInstance().addRoutine(
                new AutoRoutine("Figure 8", Autos.figure8(TrajectoryLoader.getAutoPath("Figure8"), drivetrain)));

        FieldConstants.displayOnField();
    }
}