package frc.robot.subsystems;

import java.util.Map;
import java.util.function.Supplier;

import com.techhounds.houndutil.houndlog.LogGroup;
import com.techhounds.houndutil.houndlog.LogProfileBuilder;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.enums.LogLevel;
import com.techhounds.houndutil.houndlog.loggers.DeviceLogger;
import com.techhounds.houndutil.houndlog.logitems.BooleanLogItem;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.GamePieceLocation.GamePiece;
import frc.robot.commands.RobotStates;

/**
 * The manipulator class, containing the wrist, pincer, and pole detector.
 * 
 * @author gc
 */
public class Manipulator extends SubsystemBase {
    /** The solenoid that controls the wrist of the manipulator. */
    private DoubleSolenoid wrist = new DoubleSolenoid(PneumaticsModuleType.REVPH,
            Constants.Pneumatics.WRIST[0],
            Constants.Pneumatics.WRIST[1]);

    /** The solenoid that controls the pincer to hold game pieces. */
    private DoubleSolenoid pincer = new DoubleSolenoid(PneumaticsModuleType.REVPH,
            Constants.Pneumatics.PINCERS[0],
            Constants.Pneumatics.PINCERS[1]);

    /** Beam break sensor that detects if the wingdong is hitting the pole. */
    private DigitalInput poleSwitch = new DigitalInput(Constants.DIO.POLE_SWITCH);
    private DIOSim poleSwitchSim;

    /** The ligament of the complete mechanism body that this subsystem controls. */
    private MechanismLigament2d wristLigament;

    /**
     * Initializes the manipulator.
     */
    public Manipulator(MechanismLigament2d wristLigament) {
        this.wristLigament = wristLigament;

        LoggingManager.getInstance().addGroup("Manipulator", new LogGroup(
                new DeviceLogger<DoubleSolenoid>(wrist, "Wrist",
                        LogProfileBuilder.buildDoubleSolenoidLogItems(wrist)),
                new DeviceLogger<DoubleSolenoid>(pincer, "Pincer",
                        LogProfileBuilder.buildDoubleSolenoidLogItems(pincer)),
                new BooleanLogItem("Is Pole Detected", this::isPoleDetected, LogLevel.MAIN)));

        if (RobotBase.isSimulation()) {
            poleSwitchSim = new DIOSim(poleSwitch);
            poleSwitchSim.setValue(true);

        }
    }

    /**
     * Runs every 20ms. This updates the ligament sent over NetworkTables.
     */
    @Override
    public void periodic() {
        super.periodic();
        if (wrist.get() == Value.kForward) {
            wristLigament.setAngle(0);
        } else {
            wristLigament.setAngle(90);
        }
    }

    // private boolean isSafeForWristMove(Elevator elevator) {
    // return elevator.isSafeForWrist();
    // }

    /**
     * Creates an InstantCommand that sets the wrist to the
     * down position.
     * 
     * @return the command
     */
    public CommandBase setWristDownCommand() {
        return runOnce(() -> wrist.set(Value.kForward)).withName("Wrist Down"); // untested
    }

    /**
     * Creates an InstantCommand that sets the wrist to the
     * down position.
     * 
     * @return the command
     */
    public CommandBase setWristUpCommand(Elevator elevator, LEDs leds) {
        return Commands.either(
                runOnce(() -> wrist.set(Value.kReverse)),
                RobotStates.singularErrorCommand(() -> "Error"),
                () -> true).withName("Wrist Up"); // TODO
    }

    /**
     * Creates an InstantCommand that sets the pincers to the
     * open position (space between the wedges).
     * 
     * @return the command
     */
    public CommandBase setPincersOpenCommand() {
        return runOnce(() -> pincer.set(Value.kForward)).withName("Pincers Open"); // untested
    }

    /**
     * Creates an InstantCommand that sets the pincers to the
     * closed position (no space between the wedges).
     * 
     * @return the command
     */
    public CommandBase setPincersClosedCommand() {
        return runOnce(() -> pincer.set(Value.kReverse)).withName("Pincers Closed"); // untested
    }

    /**
     * Detects whether the limit switch that detects the pole has been tripped.
     * 
     * @return true if IR beam broken by pole
     */
    public boolean isPoleDetected() {
        return !poleSwitch.get();
    }

    /**
     * Command factory that sets the pincers to the "released" position depending on
     * the GamePiece mode.
     * 
     * @param modeSupplier the object to set the pincers to "released" for
     * @return the command
     */
    public CommandBase setPincersReleasedCommand(Supplier<GamePiece> modeSupplier) {
        return Commands.select(
                Map.of(
                        GamePiece.CONE, setPincersClosedCommand(),
                        GamePiece.CUBE, setPincersOpenCommand()),
                modeSupplier::get);
    }

    /**
     * Command factory that sets the pincers to the "pincing" position depending on
     * the GamePiece mode.
     * 
     * @param mode the object to set the pincers to "pincing" for
     * @return the command
     */
    public CommandBase setPincersPincingCommand(Supplier<GamePiece> modeSupplier) {
        return Commands.select(
                Map.of(
                        GamePiece.CONE, setPincersOpenCommand(),
                        GamePiece.CUBE, setPincersClosedCommand()),
                modeSupplier::get);
    }

    /**
     * Creates a command that toggles on the pole switch to simulate that the
     * wingdong has hit the pole. This will only work in sim.
     * 
     * @return the command
     */
    public CommandBase simPoleSwitchTriggered() {
        // this is commands so that Manipulator isn't added as a requirement
        // automatically
        return Commands.runOnce(() -> poleSwitchSim.setValue(true))
                .andThen(Commands.waitSeconds(1))
                .andThen(() -> poleSwitchSim.setValue(false)).withName("Sim Pole Switch Triggered");
    }
}
