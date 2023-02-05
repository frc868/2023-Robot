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
import com.techhounds.houndutil.houndlog.logitems.DoubleLogItem;
import com.techhounds.houndutil.houndlog.logitems.StringLogItem;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import frc.robot.commands.Autos;
import frc.robot.commands.RobotStates;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Misc;
import edu.wpi.first.wpilibj2.command.PrintCommand;

/**
 * The container for the robot. Contains subsystems, OI devices, and commands.
 */
public class RobotContainer {
    private final Drivetrain drivetrain = new Drivetrain();
    private final Elbow elbow = new Elbow();
    private final Elevator elevator = new Elevator();
    private final Intake intake = new Intake();
    private final Manipulator manipulator = new Manipulator();
    private final LEDs leds = new LEDs();
    private final GridInterface gridInterface = new GridInterface();
    @SuppressWarnings("unused")
    private final Misc misc = new Misc();

    // private GamePieceMode gamePieceMode = GamePieceMode.CONE;

    /**
     * Constructs the robot container.
     */
    public RobotContainer() {
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
                new DoubleLogItem("Speed Limit", () -> Constants.Teleop.PERCENT_LIMIT,
                        LogLevel.MAIN),
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
        Controls.configureOperatorControls(1, gridInterface, intake, manipulator, elevator, elbow, leds);
        Controls.configureBackupOperatorControls(2, intake, manipulator, elevator, elbow);
    }

    public void configureAuto() {
        TrajectoryLoader.addSettings(
                new TrajectorySettings("Circle").withMaxVelocity(0.5).withMaxAcceleration(3),
                new TrajectorySettings("Figure8").withMaxVelocity(1).withMaxAcceleration(3));
        TrajectoryLoader.loadAutoPaths();

        AutoManager.getInstance().addEvent("event1", new PrintCommand("1"));
        AutoManager.getInstance().addEvent("event2", new PrintCommand("2"));
        AutoManager.getInstance().addEvent("event3", new PrintCommand("3"));
        AutoManager.getInstance().addEvent("event4", new PrintCommand("4"));

        AutoManager.getInstance().addRoutine(
                new AutoRoutine("Circle", Autos.circle(TrajectoryLoader.getAutoPath("Circle"), drivetrain)));
        AutoManager.getInstance().addRoutine(
                new AutoRoutine("Figure 8", Autos.figure8(TrajectoryLoader.getAutoPath("Figure8"), drivetrain)));

        FieldConstants.displayOnField();
    }
}