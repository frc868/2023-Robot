package frc.robot;

import java.util.Arrays;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.Function;

import com.techhounds.houndutil.houndlib.oi.CommandVirpilJoystick;
import com.techhounds.houndutil.houndlib.subsystems.BaseSwerveDrive.DriveMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Elbow.ElbowPosition;
import frc.robot.Constants.Elevator.ElevatorPosition;
import frc.robot.GamePieceLocation.GamePiece;
import frc.robot.GamePieceLocation.GridPosition;
import frc.robot.GamePieceLocation.Level;
import frc.robot.Modes.RobotState;
import frc.robot.commands.IntakingCommands;
import frc.robot.commands.ScoringCommands;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Manipulator;
import static frc.robot.Constants.Teleop.*;

public class Controls {
    private static double speedMultiplier = 1.0;

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

    public static Command setSpeedMultiplierCommand(double value) {
        return Commands.runOnce(() -> {
            speedMultiplier = value;
        });
    }

    public static void configureSingleDriverControl(int port, Drivetrain drivetrain, Intake intake,
            Manipulator manipulator, Elevator elevator, Elbow elbow) {
        CommandVirpilJoystick joystick = new CommandVirpilJoystick(port);

        BooleanConsumer safeStop = (d) -> {
            if (d) {
                elevator.holdCurrentPositionCommand().schedule();
                elbow.holdCurrentPositionCommand().schedule();
            }
        };

        drivetrain.setDefaultCommand(
                drivetrain.teleopDriveCommand(
                        () -> -joystick.getY() * INPUT_LIMIT.get() * speedMultiplier,
                        () -> -joystick.getX() * INPUT_LIMIT.get() * speedMultiplier,
                        () -> -MathUtil.applyDeadband(
                                joystick.getTwist() * INPUT_LIMIT.get()
                                        * speedMultiplier,
                                0.05)));

        joystick.stickButton().onTrue(drivetrain.resetGyroCommand());
        joystick.redButton().whileTrue(
                ScoringCommands.stowElevatorCommand(intake, manipulator, elevator, elbow)
                        .finallyDo(safeStop));

        joystick.centerBottomHatUp()
                .onTrue(drivetrain.controlledRotateCommand(Math.toRadians(0), DriveMode.FIELD_ORIENTED));
        joystick.centerBottomHatLeft()
                .onTrue(drivetrain.controlledRotateCommand(Math.toRadians(90), DriveMode.FIELD_ORIENTED));
        joystick.centerBottomHatDown()
                .onTrue(drivetrain.controlledRotateCommand(Math.toRadians(180), DriveMode.FIELD_ORIENTED));
        joystick.centerBottomHatRight()
                .onTrue(drivetrain.controlledRotateCommand(Math.toRadians(270), DriveMode.FIELD_ORIENTED));
        joystick.centerBottomHatButton().onTrue(drivetrain.wheelLockCommand());

        joystick.bottomHatLeft()
                .onTrue(Modes.setIntakeModeCommand(() -> GamePiece.CONE).ignoringDisable(true));
        joystick.bottomHatRight()
                .onTrue(Modes.setIntakeModeCommand(() -> GamePiece.CUBE).ignoringDisable(true));
        joystick.bottomHatButton().onTrue(IntakingCommands.intakePieceCommand(
                joystick.getHID()::getPinkieButton,
                intake, manipulator, elevator, elbow));

        joystick.blackThumbButton()
                .onTrue(IntakingCommands.ejectPieceCommand(
                        joystick.getHID()::getPinkieButton,
                        intake, manipulator, elevator, elbow));

        joystick.centerTopHatUp().onTrue(
                ScoringCommands.fullScoreSequenceCommand(
                        joystick.getHID()::getPinkieButton, () -> Modes.getIntakeMode().get(),
                        () -> Level.HIGH,
                        drivetrain, intake, manipulator, elevator, elbow).finallyDo(safeStop));
        joystick.centerTopHatLeft().onTrue(
                ScoringCommands.fullScoreSequenceCommand(
                        joystick.getHID()::getPinkieButton, () -> Modes.getIntakeMode().get(),
                        () -> Level.MIDDLE,
                        drivetrain, intake, manipulator, elevator, elbow).finallyDo(safeStop));
        joystick.centerTopHatDown().onTrue(
                ScoringCommands.fullScoreSequenceCommand(
                        joystick.getHID()::getPinkieButton, () -> Modes.getIntakeMode().get(),
                        () -> Level.HYBRID,
                        drivetrain, intake, manipulator, elevator, elbow).finallyDo(safeStop));

        joystick.topRightHatUp().onTrue(
                IntakingCommands.humanPlayerPickupCommand(
                        joystick.getHID()::getPinkieButton, () -> GamePiece.CONE,
                        intake, manipulator, elevator, elbow).finallyDo(safeStop));
        joystick.topRightHatDown().onTrue(
                IntakingCommands.humanPlayerStowElevatorCommand(
                        intake, manipulator, elevator, elbow).finallyDo(safeStop));

        new Trigger(() -> Modes.getRobotState() == RobotState.SEEKING).and(joystick.pinkieButton())
                .onTrue(intake.toggleRamrodsCommand());

        joystick.triggerSoftPress()
                .onTrue(setSpeedMultiplierCommand(0.5))
                .onFalse(setSpeedMultiplierCommand(1.0));
        joystick.triggerHardPress()
                .onTrue(drivetrain.setDriveModeCommand(DriveMode.ROBOT_RELATIVE))
                .onFalse(drivetrain.setDriveModeCommand(DriveMode.FIELD_ORIENTED));
    }

    public static void configureDriverControls(int port, Drivetrain drivetrain, GridInterface gridInterface,
            Intake intake,
            Manipulator manipulator, Elevator elevator, Elbow elbow) {
        CommandJoystick joystick = new CommandJoystick(port);

        drivetrain.setDefaultCommand(
                drivetrain.teleopDriveCommand(
                        () -> -joystick.getY() * INPUT_LIMIT.get(),
                        () -> -joystick.getX() * INPUT_LIMIT.get(),
                        () -> -MathUtil.applyDeadband(
                                joystick.getTwist() * INPUT_LIMIT.get() * (1),
                                0.05)));

        joystick.button(13).onTrue(drivetrain.resetGyroCommand());

        // joystick.button(3).and(joystick.button(2).negate())
        // .onTrue(Commands.parallel(
        // drivetrain.setDriveModeCommand(DriveMode.ROBOT_RELATIVE),
        // Commands.runOnce(() -> isTwistLimited = true),
        // Commands.runOnce(() -> isInputCubed = true)))
        // .onFalse(Commands.parallel(
        // drivetrain.setDriveModeCommand(DriveMode.FIELD_ORIENTED),
        // Commands.runOnce(() -> isTwistLimited = false),
        // Commands.runOnce(() -> isInputCubed = false)));
        // joystick.button(2).and(joystick.button(3))
        // .onTrue(Commands.parallel(
        // drivetrain.setDriveModeCommand(DriveMode.FIELD_ORIENTED),
        // Commands.runOnce(() -> isTwistLimited = true),
        // Commands.runOnce(() -> isInputCubed = true)))
        // .whileTrue(
        // Commands.either(
        // drivetrain.controlledRotateCommand(Math.PI, true),
        // Commands.none(),
        // () ->
        // FieldConstants.AutoDrive.isInCommunity(drivetrain.getPose())).repeatedly())
        // .onFalse(Commands.parallel(
        // Commands.runOnce(() -> isTwistLimited = false),
        // Commands.runOnce(() -> isInputCubed = false)));

        // new Trigger(() -> !Modes.getIntaking()).debounce(2).and(joystick.button(14))
        // .toggleOnTrue(intake.toggleRamrodsCommand());

        joystick.pov(0, 0, CommandScheduler.getInstance().getDefaultButtonLoop())
                .whileTrue(drivetrain.controlledRotateCommand(Math.toRadians(0), DriveMode.FIELD_ORIENTED));
        // 90 on joystick is right, while 90 CCW is left
        joystick.pov(0, 90, CommandScheduler.getInstance().getDefaultButtonLoop())
                .whileTrue(drivetrain.controlledRotateCommand(Math.toRadians(270), DriveMode.FIELD_ORIENTED));
        joystick.pov(0, 180, CommandScheduler.getInstance().getDefaultButtonLoop())
                .whileTrue(drivetrain.controlledRotateCommand(Math.toRadians(180), DriveMode.FIELD_ORIENTED));
        joystick.pov(0, 270, CommandScheduler.getInstance().getDefaultButtonLoop())
                .whileTrue(drivetrain.controlledRotateCommand(Math.toRadians(90), DriveMode.FIELD_ORIENTED));

        joystick.pov(2, 0, CommandScheduler.getInstance().getDefaultButtonLoop())
                .onTrue(intake.setRamrodsRetractedCommand());
        joystick.pov(2, 180, CommandScheduler.getInstance().getDefaultButtonLoop())
                .onTrue(intake.setRamrodsExtendedCommand());
        joystick.pov(2, 270, CommandScheduler.getInstance().getDefaultButtonLoop())
                .onTrue(Modes.setIntakeModeCommand(() -> GamePiece.CONE).ignoringDisable(true));
        joystick.pov(2, 90, CommandScheduler.getInstance().getDefaultButtonLoop())
                .onTrue(Modes.setIntakeModeCommand(() -> GamePiece.CUBE).ignoringDisable(true));

        joystick.button(9)
                .onTrue(IntakingCommands.intakePieceCommand(
                        () -> joystick.getHID().getRawButton(14),
                        intake, manipulator, elevator, elbow));

        joystick.button(6)
                .onTrue(IntakingCommands.ejectPieceCommand(
                        () -> joystick.getHID().getRawButton(14),
                        intake, manipulator, elevator, elbow));

        joystick.button(8)
                .whileTrue(drivetrain.chargeStationBalanceCommand());

        // joystick.button(5).whileTrue(drivetrain.autoDriveCommand(gridInterface));

    }

    public static void configureOperatorControls(int port1, int port2, Drivetrain drivetrain,
            GridInterface gridInterface, Intake intake,
            Manipulator manipulator, Elevator elevator, Elbow elbow) {

        BooleanConsumer safeStop = (d) -> {
            if (d) {
                elevator.holdCurrentPositionCommand().schedule();
                elbow.holdCurrentPositionCommand().schedule();
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
                        Level.HYBRID));
        getButton.apply(OperatorControls.GRIDPOS_C2)
                .onTrue(gridInterface.setLocationCommand(GamePiece.HYBRID,
                        GridPosition.MIDDLE, Level.HYBRID));
        getButton.apply(OperatorControls.GRIDPOS_C3)
                .onTrue(gridInterface.setLocationCommand(GamePiece.HYBRID,
                        GridPosition.RIGHT, Level.HYBRID));

        getButton.apply(OperatorControls.RESET)
                .onTrue(Commands.runOnce(() -> gridInterface.reset()).ignoringDisable(true));

        getButton.apply(OperatorControls.SCORE)
                .whileTrue(ScoringCommands
                        .fullScoreSequenceCommand(
                                () -> hids[OperatorControls.GAME_PIECE_DROP.hid]
                                        .getHID()
                                        .getRawButton(OperatorControls.GAME_PIECE_DROP.button),
                                (b) -> setOutput.accept(
                                        OperatorControls.GAME_PIECE_DROP, b),
                                drivetrain, gridInterface, intake, manipulator,
                                elevator, elbow)
                        .andThen(Commands.runOnce(() -> setOutput.accept(OperatorControls.SCORE,
                                true)))
                        .finallyDo(safeStop))
                .onFalse(Commands.runOnce(() -> setOutput.accept(OperatorControls.SCORE,
                        false)));

        getButton.apply(OperatorControls.STOW)
                .whileTrue(ScoringCommands.stowElevatorCommand(intake, manipulator, elevator, elbow)
                        .andThen(Commands.runOnce(() -> setOutput.accept(OperatorControls.STOW,
                                true)))
                        .finallyDo(safeStop))
                .onFalse(Commands.runOnce(() -> setOutput.accept(OperatorControls.STOW,
                        false)));

        getButton.apply(OperatorControls.HP_STOW)
                .whileTrue(IntakingCommands
                        .humanPlayerStowElevatorCommand(intake, manipulator, elevator,
                                elbow)
                        .andThen(Commands.runOnce(
                                () -> setOutput.accept(OperatorControls.HP_STOW,
                                        true)))
                        .finallyDo(safeStop))
                .onFalse(Commands.runOnce(() -> setOutput.accept(OperatorControls.HP_STOW,
                        false)));

        getButton.apply(OperatorControls.HP_CONE)
                .whileTrue(
                        IntakingCommands.humanPlayerPickupCommand(
                                () -> hids[OperatorControls.GAME_PIECE_DROP.hid]
                                        .getHID()
                                        .getRawButton(OperatorControls.GAME_PIECE_DROP.button),
                                (b) -> setOutput.accept(
                                        OperatorControls.GAME_PIECE_DROP, b),
                                () -> GamePiece.CONE,
                                intake,
                                manipulator, elevator,
                                elbow)
                                .andThen(Commands.runOnce(() -> setOutput.accept(
                                        OperatorControls.HP_CONE,
                                        true)))
                                .finallyDo(safeStop))
                .onFalse(Commands.runOnce(() -> setOutput.accept(OperatorControls.HP_CONE,
                        false)));

        getButton.apply(OperatorControls.HP_CUBE)
                .whileTrue(IntakingCommands
                        .humanPlayerPickupCommand(
                                () -> hids[OperatorControls.GAME_PIECE_DROP.hid]
                                        .getHID()
                                        .getRawButton(OperatorControls.GAME_PIECE_DROP.button),
                                (b) -> setOutput.accept(
                                        OperatorControls.GAME_PIECE_DROP, b),
                                () -> GamePiece.CUBE,
                                intake, manipulator, elevator,
                                elbow)
                        .andThen(Commands.runOnce(
                                () -> setOutput.accept(OperatorControls.HP_CONE, true)))
                        .finallyDo(safeStop))
                .onFalse(Commands.runOnce(() -> setOutput.accept(OperatorControls.HP_CUBE,
                        false)));

        getButton.apply(OperatorControls.INITIALIZE)
                .whileTrue(Modes.initializeMechanisms(intake, manipulator, elevator,
                        elbow)
                        .andThen(Commands.runOnce(
                                () -> setOutput.accept(OperatorControls.INITIALIZE,
                                        true))));

    }

    public static void configureTestingControls(int port, GridInterface gridInterface, Intake intake,
            Manipulator manipulator,
            Elevator elevator, Elbow elbow) {
        CommandXboxController xbox = new CommandXboxController(port);

        xbox.rightTrigger(0.5)
                .onTrue(Commands.parallel(
                        elevator.setOverridenSpeedCommand(() -> -0.25 * xbox.getLeftY()),
                        elbow.setOverridenSpeedCommand(() -> -0.25 * xbox.getRightY())));

        xbox.y().onTrue(elevator.moveToPositionCommand(() -> ElevatorPosition.CONE_HIGH));
        xbox.x().onTrue(elevator.moveToPositionCommand(() -> ElevatorPosition.CONE_MID));
        xbox.b().onTrue(elevator.moveToPositionCommand(() -> ElevatorPosition.SINGLE_SUBSTATION_PICKUP));
        xbox.a().onTrue(elevator.moveToPositionCommand(() -> ElevatorPosition.BOTTOM));

        xbox.povUp().onTrue(elbow.moveToPositionCommand(() -> ElbowPosition.HIGH));
        xbox.povLeft().onTrue(elbow.moveToPositionCommand(() -> ElbowPosition.MID));
        xbox.povRight().onTrue(elbow.moveToPositionCommand(() -> ElbowPosition.MID_CONE_HIGH));
        xbox.povDown().onTrue(elbow.moveToPositionCommand(() -> ElbowPosition.LOW));
    }

    public static void configureBackupOperatorControls(int port, GridInterface gridInterface, Intake intake,
            Manipulator manipulator,
            Elevator elevator, Elbow elbow) {
        CommandXboxController xbox = new CommandXboxController(port);

        xbox.rightTrigger(0.5)
                .onTrue(Commands.parallel(
                        elevator.setOverridenSpeedCommand(() -> -0.25 * xbox.getLeftY()),
                        elbow.setOverridenSpeedCommand(() -> -0.25 * xbox.getRightY())));

        xbox.x().onTrue(intake.setPassoversExtendedCommand(elevator));
        xbox.b().onTrue(intake.setPassoversRetractedCommand(elevator));
        xbox.y().onTrue(intake.setRamrodsRetractedCommand());
        xbox.a().onTrue(intake.setRamrodsExtendedCommand());

        xbox.povLeft().onTrue(manipulator.setPincersOpenCommand());
        xbox.povRight().onTrue(manipulator.setPincersClosedCommand());
        xbox.povUp().onTrue(manipulator.setWristUpCommand());
        xbox.povDown().onTrue(manipulator.setWristDownCommand());
        xbox.rightBumper().whileTrue(intake.runPassoverMotorsCommand());
        xbox.leftBumper().whileTrue(intake.reversePassoverMotorsCommand());

        xbox.leftStick().and(xbox.povUp()).onTrue(elbow.moveToPositionCommand(() -> ElbowPosition.HIGH));
        xbox.leftStick().and(xbox.povLeft()).onTrue(elbow.moveToPositionCommand(() -> ElbowPosition.MID));
        xbox.leftStick().and(xbox.povRight())
                .onTrue(elbow.moveToPositionCommand(() -> ElbowPosition.MID_CONE_HIGH));
        xbox.leftStick().and(xbox.povDown()).onTrue(elbow.moveToPositionCommand(() -> ElbowPosition.LOW));
    }

    public static void configureOverridesControls(int port1, int port2, Drivetrain drivetrain, Intake intake,
            Manipulator manipulator, Elevator elevator, Elbow elbow) {
        CommandGenericHID hid1 = new CommandGenericHID(port1);
        CommandGenericHID hid2 = new CommandGenericHID(port2);

        // hid1.button(5).whileTrue(elevator.motorOverride());
        // hid1.button(6).whileTrue(elbow.motorOverride());

        hid1.button(1).onTrue(Overrides.MANUAL_MECH_CONTROL_MODE.enableC())
                .onFalse(Overrides.MANUAL_MECH_CONTROL_MODE.disableC());
        hid1.button(2).onTrue(Overrides.ABSOLUTE_ENCODERS.enableC())
                .onFalse(Overrides.ABSOLUTE_ENCODERS.disableC());
        hid1.button(3).onTrue(drivetrain.setDriveCurrentLimitCommand(65))
                .onFalse(drivetrain.setDriveCurrentLimitCommand(40));

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

        hid1.button(1);
        hid2.button(1);
    }
}
