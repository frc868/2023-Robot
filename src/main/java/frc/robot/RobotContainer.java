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
import com.techhounds.houndutil.houndlog.loggers.Logger;
import com.techhounds.houndutil.houndlog.logitems.DoubleLogItem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import frc.robot.commands.AutoRoutineHelpers;
import frc.robot.commands.auto.Circle;
import frc.robot.commands.auto.Figure8;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * The container for the robot. Contains subsystems, OI devices, and commands.
 */
public class RobotContainer {
    private final Drivetrain drivetrain = new Drivetrain();

    /**
     * Constructs the robot container.
     */
    public RobotContainer() {
        LiveWindow.disableAllTelemetry(); // livewindow is basically deprecated. using houndlog instead.
        if (RobotBase.isSimulation()) { // prevents annoying joystick disconnected warning
            DriverStation.silenceJoystickConnectionWarning(true);
        }
        configureButtonBindings();
        configureAuto();

        LoggingManager.getInstance().addGroup("Main", new LogGroup(new Logger[] {
                new DoubleLogItem("Speed Limit", () -> Constants.Teleop.PERCENT_LIMIT, LogLevel.MAIN) }));
    }

    private void configureButtonBindings() {
        switch (Constants.CONTROLLER_TYPE) {
            case XboxController:
                CommandXboxController driverController = new CommandXboxController(Constants.OI.DRIVER_PORT);
                // Driver left joystick and right joystick, drive
                drivetrain.setDefaultCommand(
                        drivetrain.teleopDriveCommand(
                                () -> driverController.getLeftX(),
                                () -> driverController.getLeftY(),
                                () -> driverController.getRightY(),
                                () -> false));

            case FlightStick:
                CommandJoystick joystick = new CommandJoystick(0);

                drivetrain.setDefaultCommand(
                        drivetrain.teleopDriveCommand(
                                () -> -joystick.getY() * 2.0,
                                () -> -joystick.getX() * 2.0,
                                () -> -joystick.getTwist() * 2.0,
                                () -> joystick.getHID().getRawButton(1)));

                joystick.button(12)
                        .onTrue(new InstantCommand(drivetrain::zeroGyro).beforeStarting(new PrintCommand("resetting")));

                joystick.button(11).onTrue(drivetrain.turnWhileMovingCommand(true));
                joystick.button(9).onTrue(drivetrain.turnWhileMovingCommand(false));

                joystick.button(8).onTrue(
                        new InstantCommand(() -> Constants.Teleop.PERCENT_LIMIT += 0.05, drivetrain));
                joystick.button(10).onTrue(
                        new InstantCommand(() -> Constants.Teleop.PERCENT_LIMIT -= 0.05, drivetrain));

                joystick.button(-1)
                        .onTrue(AutoRoutineHelpers.generateSwervePathFollowingCommand(PathPlanner.generatePath(
                                new PathConstraints(4, 3),
                                new PathPoint(drivetrain.getPose().getTranslation(),
                                        drivetrain.getPose().getRotation()),
                                new PathPoint(new Translation2d(3.0, 3.0), Rotation2d.fromDegrees(45))), drivetrain));
        }
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
                new AutoRoutine("Circle", new Circle(TrajectoryLoader.getAutoPath("Circle"), drivetrain)));
        AutoManager.getInstance().addRoutine(
                new AutoRoutine("Figure 8", new Figure8(TrajectoryLoader.getAutoPath("Figure8"), drivetrain)));

    }
}
