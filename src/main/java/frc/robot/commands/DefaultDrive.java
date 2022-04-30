package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * The default tank drive. This could be defined as an inline command, but I
 * chose to define it as a class for verbosity.
 * 
 * @author dr
 */
public class DefaultDrive extends CommandBase {
    private Drivetrain drivetrain;
    private DoubleSupplier leftSpeed, rightSpeed;

    public DefaultDrive(Drivetrain drivetrain, DoubleSupplier leftSpeed, DoubleSupplier rightSpeed) {
        this.drivetrain = drivetrain;
        this.leftSpeed = leftSpeed;
        this.rightSpeed = rightSpeed;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.tankDrive(leftSpeed.getAsDouble(), rightSpeed.getAsDouble());
    }
}
