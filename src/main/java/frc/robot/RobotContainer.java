// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OI;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.DrivetrainRamsete;
import frc.robot.commands.RunShooter;
import frc.robot.commands.TurnToBall;
import frc.robot.commands.TurnToGoal;
import frc.robot.commands.auton.paths.FiveBall;
import frc.robot.commands.auton.paths.FourBall;
import frc.robot.commands.auton.paths.LeaveTarmac;
import frc.robot.commands.auton.paths.ThreeBall;
import frc.robot.commands.auton.paths.TwoBall;
import frc.robot.logging.LogGroup;
import frc.robot.logging.SendableLogger;
import frc.robot.sensors.Astra;
import frc.robot.sensors.Limelight;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Misc;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

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

    XboxController driverController = new XboxController(OI.DRIVER_PORT);
    XboxController operatorController = new XboxController(OI.OPERATOR_PORT);
    SendableChooser<Command> chooser = new SendableChooser<>();
    LogGroup commandViewer = new LogGroup(
            new SendableLogger[] {
                    new SendableLogger("Commands", "Intake Up", new InstantCommand(intake::setUp, intake)),
                    new SendableLogger("Commands", "Intake Down", new InstantCommand(intake::setDown, intake)),
                    new SendableLogger("Commands", "Extend Climber Locks",
                            new InstantCommand(climber::extendLock, climber)),
                    new SendableLogger("Commands", "Retract Climber Locks",
                            new InstantCommand(climber::retractLock, climber)),
                    new SendableLogger("Commands", "Extend Climber Stage 2",
                            new InstantCommand(climber::extendSecondStage, climber)),
                    new SendableLogger("Commands", "Retract Climber Stage 2",
                            new InstantCommand(climber::retractSecondStage, climber)),
                    new SendableLogger("Commands", "Run Hopper",
                            new StartEndCommand(hopper::runMotor, hopper::stopMotor, hopper)),
                    new SendableLogger("Commands", "Run Intake",
                            new StartEndCommand(intake::runMotors, intake::stop, intake)),
                    new SendableLogger("Commands", "Gatekeepers In",
                            new InstantCommand(hopper::gatekeepersIn, hopper)),
                    new SendableLogger("Commands", "Gatekeepers Out",
                            new InstantCommand(hopper::gatekeepersOut, hopper)),
                    new SendableLogger("Commands", "Run Shooter", new RunShooter(shooter, limelight)),
                    new SendableLogger("Commands", "Run Shooter Locked Speed", new RunShooter(shooter, limelight)),
                    new SendableLogger("Commands", "Turn To Goal", new TurnToGoal(drivetrain, limelight)),
                    new SendableLogger("Commands", "Turn To Ball", new TurnToBall(drivetrain, astra)),
            });

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        LiveWindow.disableAllTelemetry();
        // Configure the button bindings
        drivetrain.setDefaultCommand(
                new DefaultDrive(drivetrain, driverController::getLeftX, driverController::getRightX));
        climber.setDefaultCommand(
                new FunctionalCommand(() -> {
                }, () -> climber.setSpeed(operatorController.getLeftY()), (interrupted) -> {
                }, () -> false, climber));
        configureButtonBindings();
        configureAutonChooser();
        commandViewer.init(); // this only needs to be run once since Sendables are put on SmartDashboard
                              // declaratively (that is, they only need to be added once and don't need to be
                              // updated to work).

        // Ignore everything here, this is just a test to put on SmartDashboard
        Trajectory testTrajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                new Pose2d(3, 0, new Rotation2d(0)),
                new TrajectoryConfig(
                        Constants.Auton.MAX_VELOCITY,
                        Constants.Auton.MAX_ACCELERATION)
                                .setKinematics(drivetrain.getKinematics())
                                .addConstraint(new DifferentialDriveVoltageConstraint(
                                        new SimpleMotorFeedforward(
                                                Constants.Drivetrain.kS,
                                                Constants.Drivetrain.kV,
                                                Constants.Drivetrain.kA),
                                        drivetrain.getKinematics(),
                                        10)));
        SmartDashboard.putData(new DrivetrainRamsete(testTrajectory, drivetrain));
    }

    private void configureAutonChooser() {
        chooser.addOption("Leave Tarmac", new LeaveTarmac(drivetrain, shooter, intake, hopper, limelight));
        chooser.addOption("Two Ball", new TwoBall(drivetrain, shooter, intake, hopper, limelight));
        chooser.setDefaultOption("Three Ball",
                new ThreeBall(drivetrain, shooter, intake, hopper, limelight, astra));
        chooser.addOption("Four Ball", new FourBall(drivetrain, shooter, intake, hopper, limelight, astra));
        chooser.addOption("Five Ball", new FiveBall(drivetrain, shooter, intake, hopper, limelight, astra));
    }

    private void configureButtonBindings() {
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

    }

    public Command getAutonomousCommand() {
        return chooser.getSelected();
    }
}
