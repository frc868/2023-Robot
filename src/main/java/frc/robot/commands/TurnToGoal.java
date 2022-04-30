package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.sensors.Limelight;
import frc.robot.subsystems.Drivetrain;

/**
 * Turns to the goal using a PID loop. (NOTE: it may be useful to set a timeout
 * on this command to prevent autos getting stuck at this phase.)
 * 
 * @author dr
 */
public class TurnToGoal extends PIDCommand {
    private final Drivetrain drivetrain;

    public TurnToGoal(Drivetrain drivetrain, Limelight limelight) {
        super(new PIDController(Constants.Drivetrain.TurnToGoalPID.kP, Constants.Drivetrain.TurnToGoalPID.kI,
                Constants.Drivetrain.TurnToGoalPID.kD), limelight::getTx, 0, d -> drivetrain.tankDrive(d, -d),
                drivetrain, limelight);

        this.drivetrain = drivetrain;
        addRequirements(drivetrain, limelight);
        getController().setTolerance(0.5);
    }

    @Override
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
