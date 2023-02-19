package frc.robot;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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

public class Controls {
    public static void configureDriverControls(int port, Drivetrain drivetrain, GridInterface gridInterface,
            Intake intake,
            Manipulator manipulator, Elevator elevator, Elbow elbow, LEDs leds) {
        CommandJoystick joystick = new CommandJoystick(port);

        drivetrain.setDefaultCommand(
                drivetrain.teleopDriveCommand(
                        () -> -joystick.getY(),
                        () -> -joystick.getX(),
                        () -> -joystick.getTwist(),
                        () -> 1 - joystick.getRawAxis(5)));

        joystick.button(13).onTrue(drivetrain.zeroGyroCommand());

        joystick.button(2).onTrue(drivetrain.setSpeedModeCommand(Drivetrain.SpeedMode.ULTRA));
        joystick.button(3).and(joystick.button(5)).onTrue(drivetrain.setSpeedModeCommand(Drivetrain.SpeedMode.FAST));

        joystick.pov(2, 270, CommandScheduler.getInstance().getDefaultButtonLoop())
                .onTrue(RobotStates.setIntakeModeCommand(GamePiece.CONE).ignoringDisable(true));
        joystick.pov(2, 90, CommandScheduler.getInstance().getDefaultButtonLoop())
                .onTrue(RobotStates.setIntakeModeCommand(GamePiece.CUBE).ignoringDisable(true));

        joystick.button(14)
                .onTrue(RobotStates.intakeGamePiece(() -> joystick.getHID().getRawButton(6), intake, manipulator,
                        elevator, elbow,
                        leds));
        // joystick.button(5).whileTrue(Commands.runOnce(
        // () -> AutoManager.getInstance().getField().getObject("Traj")
        // .setTrajectory(PathPlanner.generatePath(
        // new PathConstraints(0.5, 0.5),
        // new PathPoint(drivetrain.getPose().getTranslation(),
        // drivetrain.getPose().getRotation()),
        // new PathPoint(
        // drivetrain.getPose()
        // .plus(new Transform2d(new Translation2d(1, 0),
        // new Rotation2d(0)))
        // .getTranslation(),
        // Rotation2d.fromDegrees(0)))))
        // .andThen(
        // new ProxyCommand(
        // () -> drivetrain.pathFollowingCommand(PathPlanner.generatePath(
        // new PathConstraints(0.5, 0.5),
        // new PathPoint(drivetrain.getPose().getTranslation(),
        // drivetrain.getPose().getRotation()),
        // new PathPoint(
        // drivetrain.getPose()
        // .plus(new Transform2d(
        // new Translation2d(1, 0),
        // new Rotation2d(0)))
        // .getTranslation(),
        // Rotation2d.fromDegrees(0), drivetrain.getPose().getRotation())))))
        // .andThen(() -> drivetrain.stop()));
        joystick.button(5).whileTrue(RobotStates.autoDrive(drivetrain, gridInterface, leds));

    }

    public enum OperatorControls {
        LEFT_GRID(1, 1),
        MIDDLE_GRID(1, 2),
        RIGHT_GRID(1, 3),
        GRIDPOS_A1(0, 1),
        GRIDPOS_A2(0, 2),
        GRIDPOS_A3(0, 3),
        GRIDPOS_B1(0, 7),
        GRIDPOS_B2(0, 5),
        GRIDPOS_B3(0, 6),
        GRIDPOS_C1(0, 4), //
        GRIDPOS_C2(0, 10), //
        GRIDPOS_C3(0, 9),
        RESET(0, 8), //
        SCORE(0, 11),

        SECONDARY_BUTTON(1, 4),
        STOW(1, 5),
        ELBOW_DISABLE(1, 6),
        ELEVATOR_STEP_DOWN(1, 7),

        INITIALIZE(1, 8),
        STOW_HP(1, 9),
        ELEVATOR_DISABLE(1, 10),
        ELEVATOR_STEP_UP(1, 11);

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
                    GRIDPOS_A1,
                    GRIDPOS_A2,
                    GRIDPOS_A3,
                    GRIDPOS_B1,
                    GRIDPOS_B2,
                    GRIDPOS_B3,
                    GRIDPOS_C1,
                    GRIDPOS_C2,
                    GRIDPOS_C3);
        }
    }

    public static void configureOperatorControls(int port1, int port2, GridInterface gridInterface, Intake intake,
            Manipulator manipulator, Elevator elevator, Elbow elbow, LEDs leds) {

        BooleanConsumer safeStop = (d) -> {
            if (d) {
                elevator.disable();
                elbow.disable();
            }
        };

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

        hids[OperatorControls.GRIDPOS_A1.hid].button(OperatorControls.GRIDPOS_A1.button)
                .onTrue(gridInterface.setLocationCommand(GamePiece.CONE, GridPosition.LEFT, Level.HIGH));
        hids[OperatorControls.GRIDPOS_A2.hid].button(OperatorControls.GRIDPOS_A2.button)
                .onTrue(gridInterface.setLocationCommand(GamePiece.CUBE, GridPosition.MIDDLE, Level.HIGH));
        hids[OperatorControls.GRIDPOS_A3.hid].button(OperatorControls.GRIDPOS_A3.button)
                .onTrue(gridInterface.setLocationCommand(GamePiece.CONE, GridPosition.RIGHT, Level.HIGH));
        hids[OperatorControls.GRIDPOS_B1.hid].button(OperatorControls.GRIDPOS_B1.button)
                .onTrue(gridInterface.setLocationCommand(GamePiece.CONE, GridPosition.LEFT, Level.MIDDLE));
        hids[OperatorControls.GRIDPOS_B2.hid].button(OperatorControls.GRIDPOS_B2.button)
                .onTrue(gridInterface.setLocationCommand(GamePiece.CUBE, GridPosition.MIDDLE, Level.MIDDLE));
        hids[OperatorControls.GRIDPOS_B3.hid].button(OperatorControls.GRIDPOS_B3.button)
                .onTrue(gridInterface.setLocationCommand(GamePiece.CONE, GridPosition.RIGHT, Level.MIDDLE));
        hids[OperatorControls.GRIDPOS_C1.hid].button(OperatorControls.GRIDPOS_C1.button)
                .onTrue(gridInterface.setLocationCommand(GamePiece.HYBRID, GridPosition.LEFT, Level.LOW));
        hids[OperatorControls.GRIDPOS_C2.hid].button(OperatorControls.GRIDPOS_C2.button)
                .onTrue(gridInterface.setLocationCommand(GamePiece.HYBRID, GridPosition.MIDDLE, Level.LOW));
        hids[OperatorControls.GRIDPOS_C3.hid].button(OperatorControls.GRIDPOS_C3.button)
                .onTrue(gridInterface.setLocationCommand(GamePiece.HYBRID, GridPosition.RIGHT, Level.LOW));

        hids[OperatorControls.RESET.hid].button(OperatorControls.RESET.button)
                .onTrue(Commands.runOnce(() -> gridInterface.reset()).ignoringDisable(true));

        hids[OperatorControls.SCORE.hid].button(OperatorControls.SCORE.button)
                .whileTrue(RobotStates
                        .scoreGamePiece(
                                () -> hids[OperatorControls.SECONDARY_BUTTON.hid].getHID()
                                        .getRawButton(OperatorControls.SECONDARY_BUTTON.button),
                                gridInterface, intake, manipulator,
                                elevator, elbow, leds)
                        .finallyDo(safeStop));

        hids[OperatorControls.STOW.hid].button(OperatorControls.STOW.button)
                .whileTrue(RobotStates.stowElevator(intake, manipulator, elevator, elbow, leds).finallyDo(safeStop));

        hids[OperatorControls.STOW_HP.hid].button(OperatorControls.STOW_HP.button)
                .whileTrue(RobotStates.stowElevator(intake, manipulator, elevator, elbow, leds, false)
                        .andThen(Commands.runOnce(() -> hids[OperatorControls.STOW_HP.hid].getHID()
                                .setOutput(OperatorControls.STOW_HP.button, true)))
                        .finallyDo(safeStop));

        hids[OperatorControls.INITIALIZE.hid].button(OperatorControls.INITIALIZE.button)
                .onTrue(RobotStates.initializeMechanisms(intake, manipulator, elevator, elbow, leds));

        hids[OperatorControls.ELEVATOR_STEP_UP.hid].button(OperatorControls.ELEVATOR_STEP_UP.button)
                .onTrue(elevator.setDesiredPositionDeltaCommand(0.1, intake, elbow, leds));
        hids[OperatorControls.ELEVATOR_STEP_DOWN.hid].button(OperatorControls.ELEVATOR_STEP_DOWN.button)
                .onTrue(elevator.setDesiredPositionDeltaCommand(-0.1, intake, elbow, leds));

        hids[OperatorControls.ELEVATOR_DISABLE.hid].button(OperatorControls.ELEVATOR_DISABLE.button)
                .onTrue(Commands.runOnce(elevator::disable));
        hids[OperatorControls.ELBOW_DISABLE.hid].button(OperatorControls.ELBOW_DISABLE.button)
                .onTrue(Commands.runOnce(elbow::disable));

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
    }

    public static void configureOverridesControls(int port1, int port2, Drivetrain drivetrain, Intake intake,
            Manipulator manipulator, Elevator elevator, Elbow elbow, LEDs leds) {
        CommandGenericHID hid1 = new CommandGenericHID(port1);
        CommandGenericHID hid2 = new CommandGenericHID(port2);

        // hid1.button(5).whileTrue(elevator.motorOverride());
        // hid1.button(6).whileTrue(elbow.motorOverride());

        hid1.button(15).onTrue(
                Commands.parallel(
                        Overrides.MANUAL_MECH_CONTROL_MODE.disableC(),
                        Overrides.SAFETIES_DISABLE.disableC(),
                        Overrides.SPEED_LIMITS_DISABLE.disableC(),
                        Overrides.MECH_LIMITS_DISABLE.disableC(),
                        Overrides.DRIVER_EMERGENCY_MODE.disableC()));
        hid1.button(16).onTrue(
                Commands.parallel(
                        Overrides.MANUAL_MECH_CONTROL_MODE.enableC(),
                        Overrides.SAFETIES_DISABLE.disableC(),
                        Overrides.SPEED_LIMITS_DISABLE.disableC(),
                        Overrides.MECH_LIMITS_DISABLE.disableC(),
                        Overrides.DRIVER_EMERGENCY_MODE.disableC()));
        hid1.button(17).onTrue(
                Commands.parallel(
                        Overrides.MANUAL_MECH_CONTROL_MODE.enableC(),
                        Overrides.SAFETIES_DISABLE.enableC(),
                        Overrides.SPEED_LIMITS_DISABLE.disableC(),
                        Overrides.MECH_LIMITS_DISABLE.disableC(),
                        Overrides.DRIVER_EMERGENCY_MODE.disableC()));
        hid1.button(18).onTrue(
                Commands.parallel(
                        Overrides.MANUAL_MECH_CONTROL_MODE.enableC(),
                        Overrides.SAFETIES_DISABLE.enableC(),
                        Overrides.SPEED_LIMITS_DISABLE.enableC(),
                        Overrides.MECH_LIMITS_DISABLE.enableC(),
                        Overrides.DRIVER_EMERGENCY_MODE.disableC()));
        hid1.button(19).onTrue(
                Commands.parallel(
                        Overrides.MANUAL_MECH_CONTROL_MODE.enableC(),
                        Overrides.SAFETIES_DISABLE.enableC(),
                        Overrides.SPEED_LIMITS_DISABLE.enableC(),
                        Overrides.MECH_LIMITS_DISABLE.enableC(),
                        Overrides.DRIVER_EMERGENCY_MODE.enableC()));

        hid2.button(1);
    }
}
