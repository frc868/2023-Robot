package frc.robot.subsystems;

import java.util.Map;
import java.util.function.Supplier;

import com.techhounds.houndutil.houndlog.interfaces.Log;
import com.techhounds.houndutil.houndlog.interfaces.LoggedObject;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.GamePieceLocation.GamePiece;
import static frc.robot.Constants.Manipulator.*;

/**
 * The manipulator class, containing the wrist, pincer, and pole detector.
 * 
 * @author gc
 */
@LoggedObject
public class Manipulator extends SubsystemBase {
    /** The solenoid that controls the wrist of the manipulator. */
    @Log
    private DoubleSolenoid wrist = new DoubleSolenoid(PneumaticsModuleType.REVPH, WRIST_PORTS[0], WRIST_PORTS[1]);

    /** The solenoid that controls the pincer to hold game pieces. */
    @Log
    private DoubleSolenoid pincers = new DoubleSolenoid(PneumaticsModuleType.REVPH, PINCERS_PORTS[0], PINCERS_PORTS[1]);

    /** Beam break sensor that detects if the wingdong is hitting the pole. */
    @Log
    private DigitalInput poleSwitch = new DigitalInput(POLE_SWITCH_PORT);

    private DIOSim poleSwitchSim;

    /** The ligament of the complete mechanism body that this subsystem controls. */
    private MechanismLigament2d wristLigament;

    private Supplier<Pose3d> elbowPoseSupplier;

    private Timer wristPoseTimer = new Timer();
    private Timer pincersPoseTimer = new Timer();

    /**
     * Initializes the manipulator.
     */
    public Manipulator(Supplier<Pose3d> elbowPoseSupplier, MechanismLigament2d wristLigament) {
        this.elbowPoseSupplier = elbowPoseSupplier;
        this.wristLigament = wristLigament;

        if (RobotBase.isSimulation()) {
            poleSwitchSim = new DIOSim(poleSwitch);
            poleSwitchSim.setValue(true);
        }

        new Trigger(() -> wrist.get() == Value.kForward)
                .onTrue(Commands.runOnce(wristPoseTimer::restart));
        new Trigger(() -> wrist.get() == Value.kReverse)
                .onTrue(Commands.runOnce(wristPoseTimer::restart));

        new Trigger(() -> pincers.get() == Value.kForward)
                .onTrue(Commands.runOnce(pincersPoseTimer::restart));
        new Trigger(() -> pincers.get() == Value.kReverse)
                .onTrue(Commands.runOnce(pincersPoseTimer::restart));
    }

    /**
     * Runs every 20ms. This updates the ligament sent over NetworkTables.
     */
    @Override
    public void periodic() {
        if (wrist.get() == Value.kForward) {
            wristLigament.setAngle(0);
        } else {
            wristLigament.setAngle(60);
        }
    }

    @Log
    public Pose3d getWristPose() {
        Pose3d elbowPose = elbowPoseSupplier.get();
        Pose3d wristPoseDown = elbowPose.plus(ELBOW_TO_WRIST);
        Pose3d wristPoseUp = elbowPose.plus(ELBOW_TO_WRIST)
                .plus(new Transform3d(new Translation3d(), new Rotation3d(0, -Math.PI / 2.0, 0)));

        return wristPoseDown.interpolate(wristPoseUp,
                wrist.get() == Value.kReverse
                        ? wristPoseTimer.get() / WRIST_MOVEMENT_TIME
                        : 1 - wristPoseTimer.get() / WRIST_MOVEMENT_TIME);
    }

    @Log
    public Pose3d getLeftPincerPose() {
        Pose3d leftPincerPoseClosed = getWristPose().plus(WRIST_TO_LEFT_PINCER);
        Pose3d leftPincerPoseOpen = leftPincerPoseClosed
                .plus(new Transform3d(0, PINCER_MOVEMENT_SECTION, 0, new Rotation3d()));

        return leftPincerPoseClosed.interpolate(leftPincerPoseOpen,
                pincers.get() == Value.kReverse
                        ? pincersPoseTimer.get() / PINCER_MOVEMENT_TIME
                        : 1 - pincersPoseTimer.get() / PINCER_MOVEMENT_TIME);
    }

    @Log
    public Pose3d getRightPincerPose() {
        Pose3d rightPincerPoseClosed = getWristPose().plus(WRIST_TO_RIGHT_PINCER);
        Pose3d rightPincerPoseOpen = rightPincerPoseClosed
                .plus(new Transform3d(0, -PINCER_MOVEMENT_SECTION, 0, new Rotation3d()));

        return rightPincerPoseClosed.interpolate(rightPincerPoseOpen,
                pincers.get() == Value.kReverse
                        ? pincersPoseTimer.get() / PINCER_MOVEMENT_TIME
                        : 1 - pincersPoseTimer.get() / PINCER_MOVEMENT_TIME);
    }

    public boolean getPincers() {
        return pincers.get() == Value.kForward;
    }

    /**
     * Creates an InstantCommand that sets the wrist to the
     * down position.
     * 
     * @return the command
     */
    public Command setWristDownCommand() {
        return Commands.runOnce(() -> wrist.set(Value.kForward)).withName("Wrist Down"); // untested
    }

    /**
     * Creates an InstantCommand that sets the wrist to the
     * down position.
     * 
     * @return the command
     */
    public Command setWristUpCommand() {
        return Commands.runOnce(() -> wrist.set(Value.kReverse)).withName("Wrist Up");
    }

    /**
     * Creates an InstantCommand that sets the pincers to the
     * open position (space between the wedges).
     * 
     * @return the command
     */
    public Command setPincersOpenCommand() {
        return runOnce(() -> pincers.set(Value.kReverse)).withName("Pincers Open"); // untested
    }

    /**
     * Creates an InstantCommand that sets the pincers to the
     * closed position (no space between the wedges).
     * 
     * @return the command
     */
    public Command setPincersClosedCommand() {
        return runOnce(() -> pincers.set(Value.kForward)).withName("Pincers Closed"); // untested
    }

    /**
     * Detects whether the limit switch that detects the pole has been tripped.
     * 
     * @return true if IR beam broken by pole
     */
    @Log
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
    public Command setPincersReleasedCommand(Supplier<GamePiece> modeSupplier) {
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
    public Command setPincersPincingCommand(Supplier<GamePiece> modeSupplier) {
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
    public Command simulatePoleSwitchTriggered() {
        // this is commands so that Manipulator isn't added as a requirement
        // automatically
        return Commands.runOnce(() -> poleSwitchSim.setValue(false))
                .andThen(Commands.waitSeconds(1))
                .andThen(() -> poleSwitchSim.setValue(true)).withName("Simulate Pole Switch Triggered");
    }
}
