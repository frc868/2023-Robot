package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.OI;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.auton.SwerveTrajectoryBuilder;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Misc;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * The container for the robot. Contains subsystems, OI devices, and commands.
 */
public class RobotContainer {
    private final Drivetrain drivetrain = new Drivetrain();
    @SuppressWarnings("unused")
    private final Misc misc = new Misc();

    private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter thetaSpeedLimiter = new SlewRateLimiter(3);

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
        chooser.addOption("5 Ball",
                SwerveTrajectoryBuilder.buildTrajectoryCommand(trajectories.get("5 Ball"), drivetrain));
    }

    private void configureButtonBindings() {
        switch (Constants.CONTROLLER_TYPE) {
            case XboxController:
                XboxController driverController = new XboxController(OI.DRIVER_PORT);
                // Driver left joystick and right joystick, drive
                drivetrain.setDefaultCommand(
                        new DefaultDrive(drivetrain,
                                () -> xSpeedLimiter.calculate(driverController.getLeftX()),
                                () -> ySpeedLimiter.calculate(driverController.getLeftY()),
                                () -> thetaSpeedLimiter.calculate(driverController.getRightY())));

            case FlightStick:
                Joystick joystick = new Joystick(0);

                drivetrain.setDefaultCommand(
                        new DefaultDrive(drivetrain,
                                () -> xSpeedLimiter.calculate(joystick.getX()),
                                () -> ySpeedLimiter.calculate(joystick.getY()),
                                () -> thetaSpeedLimiter.calculate(joystick.getTwist())));
        }

    }

    public Command getAutonomousCommand() {
        return chooser.getSelected();
    }
}
