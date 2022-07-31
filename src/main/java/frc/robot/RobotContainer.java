package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.houndutil.houndlog.LogGroup;
import frc.houndutil.houndlog.LoggingManager;
import frc.houndutil.houndlog.loggers.Logger;
import frc.houndutil.houndlog.loggers.SendableLogger;
import frc.robot.Constants.OI;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.RunShooter;
import frc.robot.commands.TurnToBall;
import frc.robot.commands.TurnToGoal;
import frc.robot.commands.auton.paths.driveandturn.FiveBall;
import frc.robot.commands.auton.paths.driveandturn.FourBall;
import frc.robot.commands.auton.paths.driveandturn.LeaveTarmac;
import frc.robot.commands.auton.paths.driveandturn.ThreeBall;
import frc.robot.commands.auton.paths.driveandturn.TwoBall;
import frc.robot.sensors.Astra;
import frc.robot.sensors.Limelight;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Misc;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * The container for the robot. Contains subsystems, OI devices, and commands.
 */
public class RobotContainer {
    private final Drivetrain drivetrain = new Drivetrain();
    private final Hopper hopper = new Hopper();
    private final Intake intake = new Intake();
    private final Climber climber = new Climber();
    private final Shooter shooter = new Shooter();
    private final Limelight limelight = new Limelight();
    private final Astra astra = new Astra();
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

        LoggingManager.getInstance().addGroup("Commands", new LogGroup(
                new Logger[] {
                        new SendableLogger("Intake Up", new InstantCommand(intake::setUp, intake)),
                        new SendableLogger("Intake Down", new InstantCommand(intake::setDown, intake)),
                        new SendableLogger("Extend Climber Locks",
                                new InstantCommand(climber::extendLock, climber)),
                        new SendableLogger("Retract Climber Locks",
                                new InstantCommand(climber::retractLock, climber)),
                        new SendableLogger("Extend Climber Stage 2",
                                new InstantCommand(climber::extendSecondStage, climber)),
                        new SendableLogger("Retract Climber Stage 2",
                                new InstantCommand(climber::retractSecondStage, climber)),
                        new SendableLogger("Run Hopper",
                                new StartEndCommand(hopper::runMotor, hopper::stopMotor, hopper)),
                        new SendableLogger("Run Intake",
                                new StartEndCommand(intake::runMotors, intake::stop, intake)),
                        new SendableLogger("Gatekeepers In",
                                new InstantCommand(hopper::gatekeepersIn, hopper)),
                        new SendableLogger("Gatekeepers Out",
                                new InstantCommand(hopper::gatekeepersOut, hopper)),
                        new SendableLogger("Run Shooter", new RunShooter(shooter, limelight)),
                        new SendableLogger("Run Shooter Locked Speed", new RunShooter(shooter, limelight)),
                        new SendableLogger("Turn To Goal", new TurnToGoal(drivetrain, limelight)),
                        new SendableLogger("Turn To Ball", new TurnToBall(drivetrain, astra)),
                }));
    }

    private void configureAutonChooser() {
        chooser.addOption("Leave Tarmac", new LeaveTarmac(drivetrain, shooter,
                intake, hopper, limelight));
        chooser.addOption("Two Ball", new TwoBall(drivetrain, shooter, intake,
                hopper, limelight));
        chooser.setDefaultOption("Three Ball",
                new ThreeBall(drivetrain, shooter, intake, hopper, limelight, astra));
        chooser.addOption("Four Ball", new FourBall(drivetrain, shooter, intake,
                hopper, limelight, astra));
        chooser.addOption("Five Ball", new FiveBall(drivetrain, shooter, intake,
                hopper, limelight, astra));
    }

    private void configureButtonBindings() {
        switch (Constants.CONTROLLER_TYPE) {
            case XboxController:
                XboxController driverController = new XboxController(OI.DRIVER_PORT);
                XboxController operatorController = new XboxController(OI.OPERATOR_PORT);
                // Driver left joystick and right joystick, drive
                drivetrain.setDefaultCommand(
                        new DefaultDrive(drivetrain,
                                () -> xSpeedLimiter.calculate(driverController.getLeftX()),
                                () -> ySpeedLimiter.calculate(driverController.getLeftY()),
                                () -> thetaSpeedLimiter.calculate(driverController.getRightY())));

                // Operator left joystick, climber motors
                climber.setDefaultCommand(
                        new RunCommand(() -> climber.setSpeed(operatorController.getLeftY()),
                                climber));

                // Driver button A, intake down
                new JoystickButton(driverController, Button.kA.value)
                        .whenPressed(new InstantCommand(intake::setDown, intake));
                // Driver button Y, intake up
                new JoystickButton(driverController, Button.kY.value)
                        .whenPressed(new InstantCommand(intake::setUp, intake));

                // Driver button B, climber locks extend
                new JoystickButton(driverController, Button.kB.value)
                        .whenPressed(new InstantCommand(climber::extendLock));
                // Driver button X, climber locks retract
                new JoystickButton(driverController, Button.kX.value)
                        .whenPressed(new InstantCommand(climber::retractLock));

                // Driver button Start, engage climber stage 2
                new JoystickButton(driverController, Button.kStart.value)
                        .whenPressed(new InstantCommand(climber::extendSecondStage));
                // Driver button Back, disengage climber stage 2
                new JoystickButton(driverController, Button.kBack.value)
                        .whenPressed(new InstantCommand(climber::retractSecondStage));

                // Operator button RB, run hopper and intake while held
                new JoystickButton(operatorController, Button.kRightBumper.value)
                        .whenPressed(
                                new ParallelCommandGroup(
                                        new StartEndCommand(hopper::runMotor, hopper::stopMotor, hopper),
                                        new StartEndCommand(intake::runMotors, intake::stop, intake)));
                // Operator button A, run hopper and intake in reverse while held
                new JoystickButton(operatorController, Button.kA.value)
                        .whenPressed(
                                new ParallelCommandGroup(
                                        new StartEndCommand(hopper::reverseMotor, hopper::stopMotor, hopper),
                                        new StartEndCommand(intake::reverseMotors, intake::stop, intake)));

                // Operator button X, gatekeepers out
                new JoystickButton(operatorController, Button.kX.value)
                        .whenPressed(new InstantCommand(hopper::gatekeepersOut, hopper));
                // Operator button B, gatekeepers in
                new JoystickButton(operatorController, Button.kB.value)
                        .whenPressed(new InstantCommand(hopper::gatekeepersIn, hopper));

                // Operator button LB, toggle shooter run with limelight regression
                new JoystickButton(operatorController, Button.kLeftBumper.value)
                        .toggleWhenPressed(new RunShooter(shooter, limelight));

                // Operator D-Pad North, turn to goal
                new POVButton(operatorController, 0)
                        .whenPressed(new TurnToGoal(drivetrain, limelight));
                // Operator D-Pad South, turn to ball
                new POVButton(operatorController, 180)
                        .whenPressed(new TurnToBall(drivetrain, astra));
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
