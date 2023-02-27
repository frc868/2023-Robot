package frc.robot;

import java.util.Arrays;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.Function;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.GamePieceLocation.GamePiece;
import frc.robot.GamePieceLocation.GridPosition;
import frc.robot.GamePieceLocation.Level;
import frc.robot.commands.RobotStates;
import frc.robot.commands.Scoring;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Manipulator;

public class Controls {
    public enum OperatorControls {
        LEFT_GRID(1, 1),
        MIDDLE_GRID(1, 2),
        RIGHT_GRID(1, 3),
        GRIDPOS_A1(0, 1),
        GRIDPOS_A2(0, 2),
        GRIDPOS_A3(0, 3),
        GRIDPOS_B1(0, 4),
        GRIDPOS_B2(0, 5),
        GRIDPOS_B3(0, 6),
        GRIDPOS_C1(0, 7),
        GRIDPOS_C2(0, 8),
        GRIDPOS_C3(0, 9),
        RESET(0, 10),
        SCORE(0, 11),

        GAME_PIECE_DROP(1, 4),
        HP_CUBE(1, 5),
        STOW(1, 6),
        UNBOUND1(1, 10),

        INITIALIZE(1, 7),
        HP_CONE(1, 8),
        HP_STOW(1, 9),
        UNBOUND2(1, 11);

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

    public static void configureDriverControls(int port, Drivetrain drivetrain, GridInterface gridInterface,
            Intake intake,
            Manipulator manipulator, Elevator elevator, Elbow elbow, LEDs leds) {
        CommandJoystick joystick = new CommandJoystick(port);

        drivetrain.setDefaultCommand(
                drivetrain.teleopDriveCommand(
                        () -> -joystick.getY(),
                        () -> -joystick.getX(),
                        () -> -MathUtil.applyDeadband(joystick.getTwist(), 0.05),
                        () -> 1 - joystick.getRawAxis(5)));

        joystick.button(13).onTrue(drivetrain.zeroGyroCommand());

        joystick.button(2).onTrue(drivetrain.setSpeedModeCommand(Drivetrain.SpeedMode.ULTRA));
        joystick.button(3).and(joystick.button(5)).onTrue(drivetrain.setSpeedModeCommand(Drivetrain.SpeedMode.FAST));

        joystick.pov(2, 270, CommandScheduler.getInstance().getDefaultButtonLoop())
                .onTrue(RobotStates.setIntakeModeCommand(GamePiece.CONE, leds).ignoringDisable(true));
        joystick.pov(2, 90, CommandScheduler.getInstance().getDefaultButtonLoop())
                .onTrue(RobotStates.setIntakeModeCommand(GamePiece.CUBE, leds).ignoringDisable(true));

        joystick.button(14)
                .onTrue(RobotStates.intakeGamePiece(
                        false,
                        true,
                        () -> joystick.getHID().getRawButton(6),
                        intake, manipulator, elevator, elbow, leds));

        joystick.button(8)
                .whileTrue(drivetrain.chargeStationBalanceCommand());
        joystick.button(5).whileTrue(RobotStates.autoDriveCommand(drivetrain, gridInterface));

    }

    public static void configureOperatorControls(int port1, int port2, Drivetrain drivetrain,
            GridInterface gridInterface, Intake intake,
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

        Function<OperatorControls, Trigger> getButton = (control) -> {
            return hids[control.hid].button(control.button);
        };
        BiConsumer<OperatorControls, Boolean> setOutput = (control, bool) -> {
            hids[control.hid].getHID().setOutput(control.button, bool);
        };

        gridInterface.setHIDOutputDevices(hids[0].getHID(), hids[1].getHID());

        getButton.apply(OperatorControls.LEFT_GRID)
                .onTrue(gridInterface.setGridCommand(GamePieceLocation.Grid.LEFT));
        getButton.apply(OperatorControls.MIDDLE_GRID)
                .onTrue(gridInterface.setGridCommand(GamePieceLocation.Grid.MIDDLE));
        getButton.apply(OperatorControls.RIGHT_GRID)
                .onTrue(gridInterface.setGridCommand(GamePieceLocation.Grid.RIGHT));

        getButton.apply(OperatorControls.GRIDPOS_A1)
                .onTrue(gridInterface.setLocationCommand(GamePiece.CONE, GridPosition.LEFT,
                        Level.HIGH));
        getButton.apply(OperatorControls.GRIDPOS_A2)
                .onTrue(gridInterface.setLocationCommand(GamePiece.CUBE, GridPosition.MIDDLE,
                        Level.HIGH));
        getButton.apply(OperatorControls.GRIDPOS_A3)
                .onTrue(gridInterface.setLocationCommand(GamePiece.CONE, GridPosition.RIGHT,
                        Level.HIGH));
        getButton.apply(OperatorControls.GRIDPOS_B1)
                .onTrue(gridInterface.setLocationCommand(GamePiece.CONE, GridPosition.LEFT,
                        Level.MIDDLE));
        getButton.apply(OperatorControls.GRIDPOS_B2)
                .onTrue(gridInterface.setLocationCommand(GamePiece.CUBE, GridPosition.MIDDLE,
                        Level.MIDDLE));
        getButton.apply(OperatorControls.GRIDPOS_B3)
                .onTrue(gridInterface.setLocationCommand(GamePiece.CONE, GridPosition.RIGHT,
                        Level.MIDDLE));
        getButton.apply(OperatorControls.GRIDPOS_C1)
                .onTrue(gridInterface.setLocationCommand(GamePiece.HYBRID, GridPosition.LEFT,
                        Level.LOW));
        getButton.apply(OperatorControls.GRIDPOS_C2)
                .onTrue(gridInterface.setLocationCommand(GamePiece.HYBRID,
                        GridPosition.MIDDLE, Level.LOW));
        getButton.apply(OperatorControls.GRIDPOS_C3)
                .onTrue(gridInterface.setLocationCommand(GamePiece.HYBRID,
                        GridPosition.RIGHT, Level.LOW));

        getButton.apply(OperatorControls.RESET)
                .onTrue(Commands.runOnce(() -> gridInterface.reset()).ignoringDisable(true));

        getButton.apply(OperatorControls.SCORE)
                .whileTrue(Scoring
                        .scoreGamePieceCommand(
                                true,
                                () -> hids[OperatorControls.GAME_PIECE_DROP.hid].getHID()
                                        .getRawButton(OperatorControls.GAME_PIECE_DROP.button),
                                (b) -> setOutput.accept(OperatorControls.GAME_PIECE_DROP, b),
                                drivetrain, gridInterface, intake, manipulator,
                                elevator, elbow)
                        .andThen(Commands.waitSeconds(0.5))
                        .andThen(RobotStates.stowElevatorCommand(intake, manipulator, elevator, elbow))
                        .andThen(Commands.runOnce(() -> setOutput.accept(OperatorControls.SCORE,
                                true)))
                        .finallyDo(safeStop))
                .onFalse(Commands.runOnce(() -> setOutput.accept(OperatorControls.SCORE,
                        false)));

        getButton.apply(OperatorControls.STOW)
                .whileTrue(RobotStates.stowElevatorCommand(intake, manipulator, elevator, elbow)
                        .andThen(Commands.runOnce(() -> setOutput.accept(OperatorControls.STOW,
                                true)))
                        .finallyDo(safeStop))
                .onFalse(Commands.runOnce(() -> setOutput.accept(OperatorControls.STOW,
                        false)));

        getButton.apply(OperatorControls.HP_STOW)
                .whileTrue(RobotStates.stowElevatorHPStation(intake, manipulator, elevator,
                        elbow, leds)
                        .andThen(Commands.runOnce(() -> setOutput.accept(OperatorControls.HP_STOW,
                                true)))
                        .finallyDo(safeStop))
                .onFalse(Commands.runOnce(() -> setOutput.accept(OperatorControls.HP_STOW,
                        false)));

        getButton.apply(OperatorControls.HP_CONE)
                .whileTrue(
                        RobotStates.humanPlayerPickup(GamePiece.CONE, gridInterface, intake,
                                manipulator, elevator,
                                elbow)
                                .andThen(Commands.runOnce(() -> setOutput.accept(OperatorControls.HP_CONE,
                                        true)))
                                .finallyDo(safeStop))
                .onFalse(Commands.runOnce(() -> setOutput.accept(OperatorControls.HP_CONE,
                        false)));

        getButton.apply(OperatorControls.HP_CUBE)
                .whileTrue(RobotStates.humanPlayerPickup(GamePiece.CUBE, gridInterface,
                        intake, manipulator, elevator,
                        elbow).andThen(Commands.runOnce(() -> setOutput.accept(OperatorControls.HP_CONE, true)))
                        .finallyDo(safeStop))
                .onFalse(Commands.runOnce(() -> setOutput.accept(OperatorControls.HP_CUBE,
                        false)));

        getButton.apply(OperatorControls.INITIALIZE)
                .whileTrue(RobotStates.initializeMechanisms(intake, manipulator, elevator,
                        elbow, leds)
                        .andThen(Commands.runOnce(() -> setOutput.accept(OperatorControls.INITIALIZE,
                                true))));

    }

    public static void configureBackupOperatorControls(int port, GridInterface gridInterface, Intake intake,
            Manipulator manipulator,
            Elevator elevator, Elbow elbow, LEDs leds) {
        CommandXboxController xbox = new CommandXboxController(port);

        elevator.setDefaultCommand(
                elevator.setOverridenElevatorSpeedCommand(() -> -0.25 * xbox.getLeftY(), intake,
                        elbow));
        elbow.setDefaultCommand(elbow.setOverridenElbowSpeedCommand(() -> -0.25 * xbox.getRightY()));

        xbox.x().onTrue(intake.setPassoversExtendedCommand(elevator));
        xbox.b().onTrue(intake.setPassoversRetractedCommand(elevator));
        xbox.y().onTrue(intake.setIntakeUpCommand(elevator));
        xbox.a().onTrue(intake.setIntakeDownCommand(elevator));

        xbox.povLeft().onTrue(manipulator.setPincersOpenCommand());
        xbox.povRight().onTrue(manipulator.setPincersClosedCommand());
        xbox.povUp().onTrue(manipulator.setWristUpCommand(elevator));
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

        hid1.button(1).onTrue(Overrides.MANUAL_MECH_CONTROL_MODE.enableC())
                .onFalse(Overrides.MANUAL_MECH_CONTROL_MODE.disableC());

        // hid1.button(15).onTrue(
        // Commands.parallel(
        // Overrides.MANUAL_MECH_CONTROL_MODE.disableC(),
        // Overrides.SAFETIES_DISABLE.disableC(),
        // Overrides.SPEED_LIMITS_DISABLE.disableC(),
        // Overrides.MECH_LIMITS_DISABLE.disableC(),
        // Overrides.DRIVER_EMERGENCY_MODE.disableC()));
        // hid1.button(16).onTrue(
        // Commands.parallel(
        // Overrides.MANUAL_MECH_CONTROL_MODE.enableC(),
        // Overrides.SAFETIES_DISABLE.disableC(),
        // Overrides.SPEED_LIMITS_DISABLE.disableC(),
        // Overrides.MECH_LIMITS_DISABLE.disableC(),
        // Overrides.DRIVER_EMERGENCY_MODE.disableC()));
        // hid1.button(17).onTrue(
        // Commands.parallel(
        // Overrides.MANUAL_MECH_CONTROL_MODE.enableC(),
        // Overrides.SAFETIES_DISABLE.enableC(),
        // Overrides.SPEED_LIMITS_DISABLE.disableC(),
        // Overrides.MECH_LIMITS_DISABLE.disableC(),
        // Overrides.DRIVER_EMERGENCY_MODE.disableC()));
        // hid1.button(18).onTrue(
        // Commands.parallel(
        // Overrides.MANUAL_MECH_CONTROL_MODE.enableC(),
        // Overrides.SAFETIES_DISABLE.enableC(),
        // Overrides.SPEED_LIMITS_DISABLE.enableC(),
        // Overrides.MECH_LIMITS_DISABLE.enableC(),
        // Overrides.DRIVER_EMERGENCY_MODE.disableC()));
        // hid1.button(19).onTrue(
        // Commands.parallel(
        // Overrides.MANUAL_MECH_CONTROL_MODE.enableC(),
        // Overrides.SAFETIES_DISABLE.enableC(),
        // Overrides.SPEED_LIMITS_DISABLE.enableC(),
        // Overrides.MECH_LIMITS_DISABLE.enableC(),
        // Overrides.DRIVER_EMERGENCY_MODE.enableC()));

        hid2.button(1);
    }
}
