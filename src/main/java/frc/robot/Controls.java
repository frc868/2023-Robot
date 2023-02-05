package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.print;
import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.startEnd;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.GamePieceLocation.GamePiece;
import frc.robot.GamePieceLocation.GridPosition;
import frc.robot.GamePieceLocation.Level;
import frc.robot.commands.RobotStates;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Manipulator;

public class Controls {
    public static void configureDriverControls(int port, Drivetrain drivetrain, Intake intake, Manipulator manipulator,
            Elevator elevator, Elbow elbow, LEDs leds) {
        CommandJoystick joystick = new CommandJoystick(port);

        drivetrain.setDefaultCommand(
                drivetrain.teleopDriveCommand(
                        () -> -joystick.getY() * 2.0,
                        () -> -joystick.getX() * 2.0,
                        () -> -joystick.getTwist() * 2.0));

        joystick.button(12)
                .onTrue(runOnce(drivetrain::zeroGyro).beforeStarting(print("resetting")));

        joystick.button(11).onTrue(drivetrain.turnWhileMovingCommand(true));
        joystick.button(9).onTrue(drivetrain.turnWhileMovingCommand(false));

        joystick.button(8).onTrue(
                runOnce(
                        () -> drivetrain.getSpeedMode().setLimit(drivetrain.getSpeedMode().getLimit() + 0.05),
                        drivetrain));
        joystick.button(10).onTrue(
                runOnce(
                        () -> drivetrain.getSpeedMode().setLimit(drivetrain.getSpeedMode().getLimit() - 0.05),
                        drivetrain));

        joystick.button(1).onTrue(drivetrain.setSpeedModeCommand(Drivetrain.SpeedMode.SLOW));
        joystick.button(1).onFalse(drivetrain.setSpeedModeCommand(Drivetrain.SpeedMode.FAST));
        joystick.button(2).onTrue(drivetrain.setSpeedModeCommand(Drivetrain.SpeedMode.ULTRA_FAST));
        joystick.button(2).onFalse(drivetrain.setSpeedModeCommand(Drivetrain.SpeedMode.FAST));

        joystick.button(1)
                .onTrue(drivetrain.pathFollowingCommand(PathPlanner.generatePath(
                        new PathConstraints(4, 3),
                        new PathPoint(drivetrain.getPose().getTranslation(),
                                drivetrain.getPose().getRotation()),
                        new PathPoint(new Translation2d(3.0, 3.0), Rotation2d.fromDegrees(45)))));

        joystick.button(1).onTrue(
                RobotStates.prepareToIntakeGamePiece(intake, manipulator, elevator, elbow, leds)
                        .andThen(RobotStates.intakeGamePiece(intake, manipulator, elevator, elbow, leds)));
    }

    public static void configureOperatorControls(int port, GridInterface gridInterface, Intake intake,
            Manipulator manipulator, Elevator elevator, Elbow elbow, LEDs leds) {
        CommandGenericHID hid = new CommandGenericHID(port);
        hid.button(1).onTrue(gridInterface.setGridCommand(GamePieceLocation.Grid.LEFT));
        hid.button(2).onTrue(gridInterface.setGridCommand(GamePieceLocation.Grid.MIDDLE));
        hid.button(3).onTrue(gridInterface.setGridCommand(GamePieceLocation.Grid.RIGHT));

        hid.button(4).onTrue(gridInterface.setLocationCommand(GamePiece.CONE, GridPosition.LEFT, Level.HIGH));
        hid.button(5).onTrue(gridInterface.setLocationCommand(GamePiece.CUBE, GridPosition.MIDDLE, Level.HIGH));
        hid.button(6).onTrue(gridInterface.setLocationCommand(GamePiece.CONE, GridPosition.RIGHT, Level.HIGH));
        hid.button(7).onTrue(gridInterface.setLocationCommand(GamePiece.CONE, GridPosition.LEFT, Level.MIDDLE));
        hid.button(8).onTrue(gridInterface.setLocationCommand(GamePiece.CUBE, GridPosition.MIDDLE, Level.MIDDLE));
        hid.button(9).onTrue(gridInterface.setLocationCommand(GamePiece.CONE, GridPosition.RIGHT, Level.MIDDLE));
        hid.button(10).onTrue(gridInterface.setLocationCommand(GamePiece.HYBRID, GridPosition.LEFT, Level.LOW));
        hid.button(11).onTrue(gridInterface.setLocationCommand(GamePiece.HYBRID, GridPosition.MIDDLE, Level.LOW));
        hid.button(12).onTrue(gridInterface.setLocationCommand(GamePiece.HYBRID, GridPosition.RIGHT, Level.LOW));

        hid.button(13).onTrue(runOnce(() -> gridInterface.reset()));
        hid.button(14).whileTrue(RobotStates.scoreGamePiece(gridInterface, intake, manipulator, elevator, elbow, leds)
                .andThen(RobotStates.stowElevator(intake, manipulator, elevator, elbow, leds)));

        hid.button(15).onTrue(runOnce(() -> RobotStates.setIntakeMode(GamePiece.CONE)));
        hid.button(16).onTrue(runOnce(() -> RobotStates.setIntakeMode(GamePiece.CUBE)));

    }

    public static void configureBackupOperatorControls(int port, Intake intake, Manipulator manipulator,
            Elevator elevator, Elbow elbow) {
        CommandXboxController xbox = new CommandXboxController(port);

        elevator.setDefaultCommand(run(() -> elevator.setSpeed(-xbox.getLeftY()), elevator));
        elbow.setDefaultCommand(run(() -> elbow.setSpeed(-xbox.getRightY()), elbow));

        xbox.x().onTrue(manipulator.setPincersOpenCommand());
        xbox.b().onTrue(manipulator.setPincersClosedCommand());
        xbox.y().onTrue(manipulator.setWristUpCommand());
        xbox.a().onTrue(manipulator.setWristDownCommand());
        xbox.povLeft().onTrue(intake.setPassoverRetractedCommand());
        xbox.povRight().onTrue(intake.setPassoverExtendedCommand());
        xbox.povUp().onTrue(intake.setIntakeUpCommand());
        xbox.povDown().onTrue(intake.setIntakeDownCommand());
        xbox.rightBumper().onTrue(startEnd(() -> intake.runPassoverMotors(), () -> intake.stopPassoverMotors()));
    }

    public class SpeedMode {
    }
}
