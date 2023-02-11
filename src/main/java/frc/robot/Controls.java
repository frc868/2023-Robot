package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import java.util.Arrays;
import java.util.List;

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
    public static void configureDriverControls(int port, Drivetrain drivetrain, Intake intake,
            Manipulator manipulator, Elevator elevator, Elbow elbow, LEDs leds) {
        CommandJoystick joystick = new CommandJoystick(port);

        drivetrain.setDefaultCommand(
                drivetrain.teleopDriveCommand(
                        () -> -joystick.getY(),
                        () -> -joystick.getX(),
                        () -> -joystick.getTwist()));

        joystick.button(10).onTrue(drivetrain.zeroGyroCommand());

        joystick.button(11).onTrue(drivetrain.turnWhileMovingCommand(true));
        joystick.button(9).onTrue(drivetrain.turnWhileMovingCommand(false));

        joystick.povLeft().onTrue(RobotStates.setIntakeModeCommand(GamePiece.CONE));
        joystick.povRight().onTrue(RobotStates.setIntakeModeCommand(GamePiece.CUBE));
        // joystick.button(8).onTrue(
        // runOnce(
        // () -> drivetrain.getSpeedMode().setLimit(drivetrain.getSpeedMode().getLimit()
        // + 0.05),
        // drivetrain));
        // joystick.button(10).onTrue(
        // runOnce(
        // () -> drivetrain.getSpeedMode().setLimit(drivetrain.getSpeedMode().getLimit()
        // - 0.05),
        // drivetrain));

        joystick.button(1).onTrue(drivetrain.setSpeedModeCommand(Drivetrain.SpeedMode.SLOW));
        joystick.button(2).onTrue(drivetrain.setSpeedModeCommand(Drivetrain.SpeedMode.ULTRA));
        joystick.button(1).onFalse(drivetrain.setSpeedModeCommand(Drivetrain.SpeedMode.FAST));
        joystick.button(2).onFalse(drivetrain.setSpeedModeCommand(Drivetrain.SpeedMode.FAST));

        joystick.button(3).onTrue(RobotStates.initializeMechanisms(intake, manipulator, elevator, elbow, leds));

        // joystick.button(1)
        // .onTrue());

        // joystick.button(7).onTrue(
        // RobotStates.prepareToIntakeGamePiece(intake, manipulator, elevator, elbow,
        // leds)
        // .andThen(RobotStates.intakeGamePiece(intake, manipulator, elevator, elbow,
        // leds)));
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
                .whileTrue(RobotStates.scoreGamePiece(gridInterface, intake, manipulator, elevator, elbow, leds)
                        .andThen(RobotStates.stowElevator(intake, manipulator, elevator, elbow, leds)));
    }

    public static void configureBackupOperatorControls(int port, Intake intake, Manipulator manipulator,
            Elevator elevator, Elbow elbow, LEDs leds) {
        CommandXboxController xbox = new CommandXboxController(port);

        elevator.setDefaultCommand(elevator.setOverridenElevatorSpeedCommand(() -> -xbox.getLeftY(), intake, leds));
        elbow.setDefaultCommand(elbow.setOverridenElbowSpeedCommand(() -> -xbox.getRightY()));

        xbox.x().onTrue(manipulator.setPincersOpenCommand());
        xbox.b().onTrue(manipulator.setPincersClosedCommand());
        xbox.y().onTrue(manipulator.setWristUpCommand(elevator, leds));
        xbox.a().onTrue(manipulator.setWristDownCommand());
        xbox.povLeft().onTrue(intake.setPassoverRetractedCommand(elevator, leds));
        xbox.povRight().onTrue(intake.setPassoverExtendedCommand(elevator, leds));
        xbox.povUp().onTrue(intake.setIntakeUpCommand(elevator, leds));
        xbox.povDown().onTrue(intake.setIntakeDownCommand(elevator, leds));
        xbox.rightBumper().whileTrue(intake.runPassoverMotorsCommand());
    }

    public static void configureOverridesControls(int port1, int port2, Drivetrain drivetrain, Intake intake,
            Manipulator manipulator, Elevator elevator, Elbow elbow, LEDs leds) {
        CommandGenericHID hid1 = new CommandGenericHID(port1);
        CommandGenericHID hid2 = new CommandGenericHID(port2);

        hid1.button(1).whileTrue(Overrides.operatorOverrideCommand());
        hid2.button(1);
    }
}
