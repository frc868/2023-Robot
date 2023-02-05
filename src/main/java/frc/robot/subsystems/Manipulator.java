package frc.robot.subsystems;

import java.util.Map;
import java.util.function.Supplier;

import com.techhounds.houndutil.houndlog.LogGroup;
import com.techhounds.houndutil.houndlog.LogProfileBuilder;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.loggers.DeviceLogger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
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
public class Manipulator extends SubsystemBase {
    /** The solenoid that controls the wrist of the manipulator. */
    private DoubleSolenoid wrist = new DoubleSolenoid(PneumaticsModuleType.REVPH,
            Constants.Manipulator.Pneumatics.WRIST[0],
            Constants.Manipulator.Pneumatics.WRIST[1]);

    /** The solenoid that controls the pincer to hold game pieces. */
    private DoubleSolenoid pincer = new DoubleSolenoid(PneumaticsModuleType.REVPH,
            Constants.Manipulator.Pneumatics.PINCERS[0],
            Constants.Manipulator.Pneumatics.PINCERS[1]);

    /** Beam break sensor that detects if the wingdong is hitting the pole. */
    private DigitalInput poleSwitch = new DigitalInput(Constants.Manipulator.POLE_SWITCH_PORT);

    /**
     * Initializes the manipulator.
     */
    public Manipulator() {
        LoggingManager.getInstance().addGroup("Manipulator", new LogGroup(
                new DeviceLogger<DoubleSolenoid>(wrist, "Wrist",
                        LogProfileBuilder.buildDoubleSolenoidLogItems(wrist)),
                new DeviceLogger<DoubleSolenoid>(pincer, "Pincer",
                        LogProfileBuilder.buildDoubleSolenoidLogItems(pincer))));
    }

    /**
     * Creates an InstantCommand that sets the wrist to the
     * down position.
     * 
     * @return the command
     */
    public CommandBase setWristDownCommand() {
        return runOnce(() -> wrist.set(Value.kForward)); // untested
    }

    /**
     * Creates an InstantCommand that sets the wrist to the
     * down position.
     * 
     * @return the command
     */
    public CommandBase setWristUpCommand() {
        return runOnce(() -> wrist.set(Value.kReverse)); // untested
    }

    /**
     * Creates an InstantCommand that sets the pincers to the
     * open position (space between the wedges).
     * 
     * @return the command
     */
    public CommandBase setPincersOpenCommand() {
        return runOnce(() -> pincer.set(Value.kForward)); // untested
    }

    /**
     * Creates an InstantCommand that sets the pincers to the
     * closed position (no space between the wedges).
     * 
     * @return the command
     */
    public CommandBase setPincersClosedCommand() {
        return runOnce(() -> pincer.set(Value.kReverse)); // untested
    }

    /**
     * Detects whether the limit switch that detects the pole has been tripped.
     * 
     * @return true if IR beam broken by pole
     */
    public boolean isPoleDetected() {
        return poleSwitch.get(); // untested
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
}
