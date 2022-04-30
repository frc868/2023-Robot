package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.sensors.Astra;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Constants;

/**
 * Turns to the nearest ball of our alliance color using the camera and a PID
 * loop.
 * 
 * @author dr
 */
public class TurnToBall extends PIDCommand {
    private final Drivetrain drivetrain;

    public TurnToBall(Drivetrain drivetrain, Astra astra) {
        super(new PIDController(Constants.Drivetrain.TurnToBallPID.kP, Constants.Drivetrain.TurnToBallPID.kI,
                Constants.Drivetrain.TurnToBallPID.kD), () -> astra.getTx(0), 0, d -> drivetrain.tankDrive(d, -d));

        this.drivetrain = drivetrain;
        getController().setTolerance(0.5);
        addRequirements(drivetrain, astra);
    }

    public void initialize() {
        this.drivetrain.resetEncoders();
        this.drivetrain.stop();
        super.initialize();
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }
}
