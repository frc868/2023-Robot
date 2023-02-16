package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Commands;
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
import frc.robot.subsystems.Elbow.ElbowPosition;
import frc.robot.subsystems.Elevator.ElevatorPosition;

public class Controls {
    public static void configureDriverControls(int port, Drivetrain drivetrain, Intake intake,
            Manipulator manipulator, Elevator elevator, Elbow elbow, LEDs leds) {
        CommandJoystick joystick = new CommandJoystick(port);

        drivetrain.setDefaultCommand(
                drivetrain.teleopDriveCommand(
                        () -> -joystick.getY(),
                        () -> -joystick.getX(),
                        () -> -joystick.getTwist(),
                        () -> 1 - joystick.getRawAxis(5)));

        joystick.button(6).onTrue(drivetrain.zeroGyroCommand());

        joystick.button(11).onTrue(drivetrain.turnWhileMovingCommand(true));
        joystick.button(9).onTrue(drivetrain.turnWhileMovingCommand(false));

        joystick.povLeft().onTrue(RobotStates.setIntakeModeCommand(GamePiece.CONE));
        joystick.povRight().onTrue(RobotStates.setIntakeModeCommand(GamePiece.CUBE));

        joystick.button(2).onTrue(drivetrain.setSpeedModeCommand(Drivetrain.SpeedMode.ULTRA));
        joystick.button(3).and(joystick.button(5)).onTrue(drivetrain.setSpeedModeCommand(Drivetrain.SpeedMode.FAST));

        joystick.povUp().onTrue(elbow.setDesiredPositionCommand(ElbowPosition.HIGH, elevator));
        joystick.povLeft().onTrue(elbow.setDesiredPositionCommand(ElbowPosition.MID, elevator));
        joystick.povDown().onTrue(elbow.setDesiredPositionCommand(ElbowPosition.LOW, elevator));
        joystick.button(14).onTrue(Commands.runOnce(elbow::disable));

        // joystick.button(3).onTrue(RobotStates.initializeMechanisms(intake,
        // manipulator, elevator, elbow, leds));

        // joystick.button(1)
        // .onTrue());

        joystick.button(7)
                .onTrue(RobotStates.intakeGamePiece(() -> joystick.getHID().getRawButton(14), intake, manipulator,
                        elevator, elbow,
                        leds));
    }

    public enum OperatorControls {
        LEFT_GRID(0, 1),
        MIDDLE_GRID(0, 2),
        RIGHT_GRID(0, 3),
        GRIDPOS_LEFT_HIGH(0, 4),
        GRIDPOS_MIDDLE_HIGH(0, 5),
        GRIDPOS_RIGHT_HIGH(0, 6),
        GRIDPOS_LEFT_MIDDLE(0, 7),
        GRIDPOS_MIDDLE_MIDDLE(0, 8),
        GRIDPOS_RIGHT_MIDDLE(0, 9),
        GRIDPOS_LEFT_LOW(0, 10),
        GRIDPOS_MIDDLE_LOW(0, 11),
        GRIDPOS_RIGHT_LOW(1, 1),
        RESET(1, 2),
        SCORE(1, 3),
        UNBOUND1(1, 4),
        UNBOUND2(1, 5),
        UNBOUND3(1, 6),
        UNBOUND4(1, 7),
        UNBOUND5(1, 8),
        UNBOUND6(1, 9);

        public final int hid;
        public final int button;

        private OperatorControls(int hid, int button) {
            this.hid = hid;
            this.button = button;
        }

        public static List<OperatorControls> getGridControls() {
            return Arrays.asList(
                    LEFT_GRID,
                    MIDDLE_GRID,
                    RIGHT_GRID,
                    GRIDPOS_LEFT_HIGH,
                    GRIDPOS_MIDDLE_HIGH,
                    GRIDPOS_RIGHT_HIGH,
                    GRIDPOS_LEFT_MIDDLE,
                    GRIDPOS_MIDDLE_MIDDLE,
                    GRIDPOS_RIGHT_MIDDLE,
                    GRIDPOS_LEFT_LOW,
                    GRIDPOS_MIDDLE_LOW,
                    GRIDPOS_RIGHT_LOW);
        }
    }

    public static void configureOperatorControls(int port1, int port2, GridInterface gridInterface, Intake intake,
            Manipulator manipulator, Elevator elevator, Elbow elbow, LEDs leds) {

        CommandGenericHID[] hids = new CommandGenericHID[] {
                new CommandGenericHID(port1),
                new CommandGenericHID(port2) };

        gridInterface.setHIDOutputDevices(hids[0].getHID(), hids[1].getHID());

        hids[OperatorControls.LEFT_GRID.hid].button(OperatorControls.LEFT_GRID.button)
                .onTrue(gridInterface.setGridCommand(GamePieceLocation.Grid.LEFT));
        hids[OperatorControls.MIDDLE_GRID.hid].button(OperatorControls.MIDDLE_GRID.button)
                .onTrue(gridInterface.setGridCommand(GamePieceLocation.Grid.MIDDLE));
        hids[OperatorControls.RIGHT_GRID.hid].button(OperatorControls.RIGHT_GRID.button)
                .onTrue(gridInterface.setGridCommand(GamePieceLocation.Grid.RIGHT));

        hids[OperatorControls.GRIDPOS_LEFT_HIGH.hid].button(OperatorControls.GRIDPOS_LEFT_HIGH.button)
                .onTrue(gridInterface.setLocationCommand(GamePiece.CONE, GridPosition.LEFT, Level.HIGH));
        hids[OperatorControls.GRIDPOS_MIDDLE_HIGH.hid].button(OperatorControls.GRIDPOS_MIDDLE_HIGH.button)
                .onTrue(gridInterface.setLocationCommand(GamePiece.CUBE, GridPosition.MIDDLE, Level.HIGH));
        hids[OperatorControls.GRIDPOS_RIGHT_HIGH.hid].button(OperatorControls.GRIDPOS_RIGHT_HIGH.button)
                .onTrue(gridInterface.setLocationCommand(GamePiece.CONE, GridPosition.RIGHT, Level.HIGH));
        hids[OperatorControls.GRIDPOS_LEFT_MIDDLE.hid].button(OperatorControls.GRIDPOS_LEFT_MIDDLE.button)
                .onTrue(gridInterface.setLocationCommand(GamePiece.CONE, GridPosition.LEFT, Level.MIDDLE));
        hids[OperatorControls.GRIDPOS_MIDDLE_MIDDLE.hid].button(OperatorControls.GRIDPOS_MIDDLE_MIDDLE.button)
                .onTrue(gridInterface.setLocationCommand(GamePiece.CUBE, GridPosition.MIDDLE, Level.MIDDLE));
        hids[OperatorControls.GRIDPOS_RIGHT_MIDDLE.hid].button(OperatorControls.GRIDPOS_RIGHT_MIDDLE.button)
                .onTrue(gridInterface.setLocationCommand(GamePiece.CONE, GridPosition.RIGHT, Level.MIDDLE));
        hids[OperatorControls.GRIDPOS_LEFT_LOW.hid].button(OperatorControls.GRIDPOS_LEFT_LOW.button)
                .onTrue(gridInterface.setLocationCommand(GamePiece.HYBRID, GridPosition.LEFT, Level.LOW));
        hids[OperatorControls.GRIDPOS_MIDDLE_LOW.hid].button(OperatorControls.GRIDPOS_MIDDLE_LOW.button)
                .onTrue(gridInterface.setLocationCommand(GamePiece.HYBRID, GridPosition.MIDDLE, Level.LOW));
        hids[OperatorControls.GRIDPOS_RIGHT_LOW.hid].button(OperatorControls.GRIDPOS_RIGHT_LOW.button)
                .onTrue(gridInterface.setLocationCommand(GamePiece.HYBRID, GridPosition.RIGHT, Level.LOW));

        hids[OperatorControls.RESET.hid].button(OperatorControls.RESET.button)
                .onTrue(runOnce(() -> gridInterface.reset()));
        hids[OperatorControls.SCORE.hid].button(OperatorControls.SCORE.button)
                .whileTrue(RobotStates
                        .scoreGamePiece(() -> hids[0].getHID().getRawButton(1), gridInterface, intake, manipulator,
                                elevator, elbow, leds)
                        .andThen(RobotStates.stowElevator(intake, manipulator, elevator, elbow, leds)));
    }

    public static void configureBackupOperatorControls(int port, GridInterface gridInterface, Intake intake,
            Manipulator manipulator,
            Elevator elevator, Elbow elbow, LEDs leds) {
        CommandXboxController xbox = new CommandXboxController(port);

        // elevator.setDefaultCommand(
        // elevator.setOverridenElevatorSpeedCommand(() -> -xbox.getLeftY(), intake,
        // elbow, leds));
        // elevator.setDefaultCommand(Commands.run(() ->
        // elevator.setSpeed(-xbox.getLeftY()), elevator));
        // elbow.setDefaultCommand(elbow.setOverridenElbowSpeedCommand(() ->
        // -xbox.getRightY()));
        // elbow.setDefaultCommand(Commands.run(() -> elbow.setSpeed(-0.25 *
        // xbox.getRightY()), elbow));
        // elbow.setDefaultCommand(elbow.setOverridenElbowSpeedCommand(() ->
        // -xbox.getRightY()));

        xbox.x().onTrue(intake.setPassoversExtendedCommand(elevator, leds));
        xbox.b().onTrue(intake.setPassoversRetractedCommand(elevator, leds));
        xbox.y().onTrue(intake.setIntakeUpCommand(elevator, leds));
        xbox.a().onTrue(intake.setIntakeDownCommand(elevator, leds));

        xbox.povLeft().onTrue(manipulator.setPincersOpenCommand());
        xbox.povRight().onTrue(manipulator.setPincersClosedCommand());
        xbox.povUp().onTrue(manipulator.setWristUpCommand(elevator, leds));
        xbox.povDown().onTrue(manipulator.setWristDownCommand());
        xbox.rightBumper().whileTrue(intake.runPassoverMotorsCommand());
        xbox.leftBumper().whileTrue(intake.reversePassoverMotorsCommand());

        CommandXboxController xbox2 = new CommandXboxController(4);
        xbox2.a().onTrue(elevator.setDesiredPositionCommand(ElevatorPosition.CONE_MID, intake, elbow, leds));
        xbox2.y().onTrue(elevator.setDesiredPositionCommand(ElevatorPosition.CONE_HIGH, intake, elbow, leds));
        xbox2.b().onTrue(elevator.setDesiredPositionCommand(ElevatorPosition.CONE_LOW, intake, elbow, leds));
        xbox2.leftBumper().onTrue(
                RobotStates.scoreGamePiece(() -> true, gridInterface, intake, manipulator, elevator, elbow, leds));

        xbox2.povUp().onTrue(elbow.setDesiredPositionCommand(ElbowPosition.HIGH, elevator));
        xbox2.povLeft().onTrue(elbow.setDesiredPositionCommand(ElbowPosition.MID, elevator));
        xbox2.povDown().onTrue(elbow.setDesiredPositionCommand(ElbowPosition.LOW, elevator));

        xbox2.x().onTrue(Commands.runOnce(elevator::disable));
    }

    public static void configureOverridesControls(int port1, int port2, Drivetrain drivetrain, Intake intake,
            Manipulator manipulator, Elevator elevator, Elbow elbow, LEDs leds) {
        CommandGenericHID hid1 = new CommandGenericHID(port1);
        CommandGenericHID hid2 = new CommandGenericHID(port2);

        hid1.button(1).whileTrue(Overrides.operatorOverrideCommand());
        hid2.button(1);
    }
}
