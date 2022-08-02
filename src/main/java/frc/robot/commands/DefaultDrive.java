package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * The default way to drive swerve, taking in xSpeed, ySpeed, and thetaSpeed.
 * This will be field-oriented.
 * 
 * @author dr
 */
public class DefaultDrive extends CommandBase {
    private Drivetrain drivetrain;
    private DoubleSupplier xSpeed, ySpeed, thetaSpeed;

    public DefaultDrive(Drivetrain drivetrain, DoubleSupplier xSpeed, DoubleSupplier ySpeed,
            DoubleSupplier thetaSpeed) {
        this.drivetrain = drivetrain;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.thetaSpeed = thetaSpeed;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.drive(
                xSpeed.getAsDouble() * Constants.Teleop.POWER_LIMIT,
                ySpeed.getAsDouble() * Constants.Teleop.POWER_LIMIT,
                thetaSpeed.getAsDouble() * Constants.Teleop.POWER_LIMIT,
                drivetrain.getDriveMode());
    }
}
