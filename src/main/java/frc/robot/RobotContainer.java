package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.TurnWheelsToAngle;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * The container for the robot. Contains subsystems, OI devices, and commands.
 */
public class RobotContainer {
    private final Drivetrain drivetrain = new Drivetrain();

    SendableChooser<Command> chooser = new SendableChooser<>();
    HashMap<String, Trajectory> trajectories = new HashMap<String, Trajectory>();

    /**
     * Constructs the robot container.
     */
    public RobotContainer() {
        LiveWindow.disableAllTelemetry(); // livewindow is basically deprecated. using houndlog instead.
        configureButtonBindings();
        configureAutonChooser();
        // loadTrajectories(); // no trajectories just yet.

    }

    private void configureAutonChooser() {
        // chooser.addOption("5 Ball",
        // SwerveTrajectoryBuilder.buildTrajectoryCommand(trajectories.get("5 Ball"),
        // drivetrain));
    }

    private void configureButtonBindings() {
        switch (Constants.CONTROLLER_TYPE) {
            case XboxController:
                XboxController driverController = new XboxController(Constants.OI.DRIVER_PORT);
                // Driver left joystick and right joystick, drive
                drivetrain.setDefaultCommand(
                        new DefaultDrive(
                                () -> driverController.getLeftX(),
                                () -> driverController.getLeftY(),
                                () -> driverController.getRightY(),
                                () -> false,
                                drivetrain));

            case FlightStick:
                Joystick joystick = new Joystick(0);

                drivetrain.setDefaultCommand(
                        new DefaultDrive(
                                () -> joystick.getY() * 2.0, // because flight stick goes -0.5 to 0.5
                                () -> joystick.getX() * 2.0,
                                () -> joystick.getTwist() * 2.0,
                                () -> joystick.getRawButton(1),
                                drivetrain));

                new JoystickButton(joystick, 7).whenPressed(new InstantCommand(drivetrain::resetGyroAngle));

                new JoystickButton(joystick, 4)
                        .whenPressed(new InstantCommand(() -> Constants.Teleop.PERCENT_LIMIT -= 0.05));
                new JoystickButton(joystick, 6)
                        .whenPressed(new InstantCommand(() -> Constants.Teleop.PERCENT_LIMIT += 0.05));

                new POVButton(joystick, 0).whenPressed(new TurnWheelsToAngle(0.0 * Math.PI / 4.0, drivetrain));
                new POVButton(joystick, 45).whenPressed(new TurnWheelsToAngle(1.0 * Math.PI / 4.0, drivetrain));
                new POVButton(joystick, 90).whenPressed(new TurnWheelsToAngle(2.0 * Math.PI / 4.0, drivetrain));
                new POVButton(joystick, 135).whenPressed(new TurnWheelsToAngle(3.0 * Math.PI / 4.0, drivetrain));
                new POVButton(joystick, 180).whenPressed(new TurnWheelsToAngle(4.0 * Math.PI / 4.0, drivetrain));
                new POVButton(joystick, 225).whenPressed(new TurnWheelsToAngle(5.0 * Math.PI / 4.0, drivetrain));
                new POVButton(joystick, 270).whenPressed(new TurnWheelsToAngle(6.0 * Math.PI / 4.0, drivetrain));
                new POVButton(joystick, 315).whenPressed(new TurnWheelsToAngle(7.0 * Math.PI / 4.0, drivetrain));

        }

    }

    public Command getAutonomousCommand() {
        return chooser.getSelected();
    }
}
