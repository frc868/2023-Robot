package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.TurnWheelsToAngle;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

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
                CommandXboxController driverController = new CommandXboxController(Constants.OI.DRIVER_PORT);
                // Driver left joystick and right joystick, drive
                drivetrain.setDefaultCommand(
                        new DefaultDrive(
                                () -> driverController.getLeftX(),
                                () -> driverController.getLeftY(),
                                () -> driverController.getRightY(),
                                () -> false,
                                drivetrain));

            case FlightStick:
                CommandJoystick joystick = new CommandJoystick(0);

                drivetrain.setDefaultCommand(
                        new DefaultDrive(
                                () -> joystick.getY() * 2.0, // because flight stick goes -0.5 to 0.5
                                () -> joystick.getX() * 2.0,
                                () -> joystick.getTwist() * 2.0,
                                () -> joystick.getHID().getRawButton(1),
                                drivetrain));

                joystick.button(7).onTrue(new InstantCommand(drivetrain::zeroGyro));

                joystick.button(4).onTrue(new InstantCommand(() -> Constants.Teleop.PERCENT_LIMIT -= 0.05));
                joystick.button(6).onTrue(new InstantCommand(() -> Constants.Teleop.PERCENT_LIMIT += 0.05));

                for (int angle = 0; angle <= 315; angle += 45)
                    joystick.pov(angle).onTrue(new TurnWheelsToAngle(Math.toRadians(angle), drivetrain));

        }

    }

    public Command getAutonomousCommand() {
        return chooser.getSelected();
    }
}
