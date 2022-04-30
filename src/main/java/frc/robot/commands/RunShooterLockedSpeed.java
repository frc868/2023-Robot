package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sensors.Limelight;
import frc.robot.subsystems.Shooter;

/**
 * Runs the shooter at a locked speed. This is useful when you want to set the
 * speed of the shooter based on the limelight once, then hold that setpoint
 * until you're done shooting. Used in auton to prevent balls passing in front
 * of the limelight from affecting the setpoint.
 * 
 * @author dr
 */
public class RunShooterLockedSpeed extends CommandBase {
    private Shooter shooter;
    private Limelight limelight;

    public RunShooterLockedSpeed(Shooter shooter, Limelight limelight) {
        this.shooter = shooter;
        this.limelight = limelight;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setSetpoint(limelight.calcShooterSpeed());
        shooter.enable();
    }

    @Override
    public void execute() {
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
