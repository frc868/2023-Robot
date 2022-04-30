package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sensors.Limelight;
import frc.robot.subsystems.Shooter;

/**
 * Runs the shooter. Now that the shooter is a PIDSubsystem, all that needs to
 * happen is to run the {@code enable()} method.
 */
public class RunShooter extends CommandBase {
    private Shooter shooter;
    private Limelight limelight;

    public RunShooter(Shooter shooter, Limelight limelight) {
        this.shooter = shooter;
        this.limelight = limelight;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.enable();
    }

    @Override
    public void execute() {
        shooter.setSetpoint(limelight.calcShooterSpeed());
    }

    @Override
    public void end(boolean interrupted) {
        shooter.disable();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
