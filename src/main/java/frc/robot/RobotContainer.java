package frc.robot;

import com.techhounds.houndutil.houndlib.auto.AutoManager;
import com.techhounds.houndutil.houndlib.auto.PPAutoRoutine;
import com.techhounds.houndutil.houndlib.auto.trajectoryloader.TrajectoryLoader;
import com.techhounds.houndutil.houndlib.auto.trajectoryloader.TrajectorySettings;
import com.techhounds.houndutil.houndlog.LogGroup;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.enums.LogLevel;
import com.techhounds.houndutil.houndlog.loggers.Logger;
import com.techhounds.houndutil.houndlog.logitems.DoubleLogItem;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import frc.robot.commands.TeleopDrive;
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

        LoggingManager.getInstance().addGroup("Main", new LogGroup(
                new Logger[] {
                        new DoubleLogItem("Speed Limit", () -> Constants.Teleop.PERCENT_LIMIT, LogLevel.MAIN)
                }));
    }

    private void configureButtonBindings() {
        switch (Constants.CONTROLLER_TYPE) {
            case XboxController:
                CommandXboxController driverController = new CommandXboxController(
                        Constants.OI.DRIVER_PORT);
                // Driver left joystick and right joystick, drive
                drivetrain.setDefaultCommand(
                        new TeleopDrive(
                                () -> driverController.getLeftX(),
                                () -> driverController.getLeftY(),
                                () -> driverController.getRightY(),
                                () -> false,
                                drivetrain));

            case FlightStick:
                CommandJoystick joystick = new CommandJoystick(0);

                drivetrain.setDefaultCommand(
                        new TeleopDrive(
                                () -> -joystick.getY() * 2.0, // because flight stick goes -0.5 to 0.5
                                () -> -joystick.getX() * 2.0,
                                () -> -joystick.getTwist() * 2.0,
                                () -> joystick.getHID().getRawButton(1),
                                drivetrain));

                joystick.button(12).onTrue(
                        new InstantCommand(drivetrain::zeroGyro)
                                .beforeStarting(new PrintCommand("resetting")));

                joystick.button(11).onTrue(drivetrain.turnWhileMovingCommand(true));
                joystick.button(9).onTrue(drivetrain.turnWhileMovingCommand(false));

                joystick.button(8).onTrue(new InstantCommand(
                        () -> Constants.Teleop.PERCENT_LIMIT += 0.05, drivetrain));
                joystick.button(10)
                        .onTrue(new InstantCommand(() -> Constants.Teleop.PERCENT_LIMIT -= 0.05,
                                drivetrain));
        }
    }

    public void configureAuto() {
        TrajectoryLoader.addSettings(
                new TrajectorySettings("Circle").withMaxVelocity(0.5).withMaxAcceleration(3),
                new TrajectorySettings("Figure8").withMaxVelocity(1).withMaxAcceleration(3));
        TrajectoryLoader.loadAutoPaths();

        AutoManager.getInstance().addRoutine(
                new PPAutoRoutine("Circle",
                        new Circle(TrajectoryLoader.getAutoPath("Circle"), drivetrain)));
        AutoManager.getInstance().addRoutine(
                new PPAutoRoutine("Figure 8",
                        new Figure8(TrajectoryLoader.getAutoPath("Figure8"), drivetrain)));
    }
}
