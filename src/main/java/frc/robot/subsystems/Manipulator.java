package frc.robot.subsystems;

import java.util.Map;
import java.util.function.Supplier;

import com.techhounds.houndutil.houndlog.interfaces.Log;
import com.techhounds.houndutil.houndlog.interfaces.LoggedObject;
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

/**
 * The manipulator class, containing the wrist, pincer, and pole detector.
 * 
 * @author gc
 */
@LoggedObject
public class Manipulator extends SubsystemBase {
    /** The solenoid that controls the wrist of the manipulator. */
    @Log(name = "Wrist Solenoid")
    private DoubleSolenoid wrist = new DoubleSolenoid(PneumaticsModuleType.REVPH,
            Constants.Pneumatics.WRIST_PORTS[0],
            Constants.Pneumatics.WRIST_PORTS[1]);

    /** The solenoid that controls the pincer to hold game pieces. */
    @Log(name = "Pincers Solenoid")
    private DoubleSolenoid pincers = new DoubleSolenoid(PneumaticsModuleType.REVPH,
            Constants.Pneumatics.PINCERS_PORTS[0],
            Constants.Pneumatics.PINCERS_PORTS[1]);

    /** Beam break sensor that detects if the wingdong is hitting the pole. */
    @Log(name = "Pole Switch")
    private DigitalInput poleSwitch = new DigitalInput(Constants.DIO.POLE_SWITCH);
    private DIOSim poleSwitchSim;

    /** The ligament of the complete mechanism body that this subsystem controls. */
    private MechanismLigament2d wristLigament;

    /**
     * Initializes the manipulator.
     */
    public Manipulator(MechanismLigament2d wristLigament) {
        this.wristLigament = wristLigament;

        if (RobotBase.isSimulation()) {
            poleSwitchSim = new DIOSim(poleSwitch);
            poleSwitchSim.setValue(false);
        }
    }

    /**
     * Runs every 20ms. This updates the ligament sent over NetworkTables.
     */
    @Override
    public void periodic() {
        super.periodic();
        if (wrist.get() == Value.kReverse) {
            wristLigament.setAngle(0);
        } else {
            wristLigament.setAngle(60);
        }
    }

    // private boolean isSafeForWristMove(Elevator elevator) {
    // return elevator.isSafeForWrist();
    // }

    public boolean getPincers() {
        return pincers.get() == Value.kForward;
    }

    /**
     * Creates an InstantCommand that sets the wrist to the
     * down position.
     * 
     * @return the command
     */
    public CommandBase setWristDownCommand() {
        return Commands.runOnce(() -> wrist.set(Value.kReverse)).withName("Wrist Down"); // untested
    }

    /**
     * Creates an InstantCommand that sets the wrist to the
     * down position.
     * 
     * @return the command
     */
    public CommandBase setWristUpCommand() {
        return Commands.runOnce(() -> wrist.set(Value.kForward)).withName("Wrist Up");
    }

    /**
     * Creates an InstantCommand that sets the pincers to the
     * open position (space between the wedges).
     * 
     * @return the command
     */
    public CommandBase setPincersOpenCommand() {
        return runOnce(() -> pincers.set(Value.kReverse)).withName("Pincers Open"); // untested
    }

    /**
     * Creates an InstantCommand that sets the pincers to the
     * closed position (no space between the wedges).
     * 
     * @return the command
     */
    public CommandBase setPincersClosedCommand() {
        return runOnce(() -> pincers.set(Value.kForward)).withName("Pincers Closed"); // untested
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
    public CommandBase simulatePoleSwitchTriggered() {
        // this is commands so that Manipulator isn't added as a requirement
        // automatically
        return Commands.runOnce(() -> poleSwitchSim.setValue(false))
                .andThen(Commands.waitSeconds(1))
                .andThen(() -> poleSwitchSim.setValue(true)).withName("Simulate Pole Switch Triggered");
    }
}
