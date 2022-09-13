package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * The default way to drive swerve, taking in xSpeed, ySpeed, and thetaSpeed.
 * This will be field-oriented.
 * 
 * @author dr
 */
public class DefaultDrive extends CommandBase {
    private Drivetrain drivetrain;
    private DoubleSupplier xSpeedSupplier, ySpeedSupplier, thetaSpeedSupplier;
    private BooleanSupplier slowModeSupplier;

    private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(5);
    private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(5);
    private final SlewRateLimiter thetaSpeedLimiter = new SlewRateLimiter(5);

    public DefaultDrive(DoubleSupplier xSpeedSupplier, DoubleSupplier ySpeedSupplier,
            DoubleSupplier thetaSpeedSupplier, BooleanSupplier slowModeSupplier, Drivetrain drivetrain) {
        this.xSpeedSupplier = xSpeedSupplier;
        this.ySpeedSupplier = ySpeedSupplier;
        this.thetaSpeedSupplier = thetaSpeedSupplier;
        this.slowModeSupplier = slowModeSupplier;
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double xSpeed = xSpeedSupplier.getAsDouble();
        double ySpeed = ySpeedSupplier.getAsDouble();
        double thetaSpeed = thetaSpeedSupplier.getAsDouble();

        xSpeed = MathUtil.applyDeadband(xSpeed, 0.05);
        ySpeed = MathUtil.applyDeadband(ySpeed, 0.05);
        thetaSpeed = MathUtil.applyDeadband(thetaSpeed, 0.05);

        if (Constants.Teleop.IS_RATE_LIMITED) {
            xSpeed = xSpeedLimiter.calculate(xSpeed);
            ySpeed = ySpeedLimiter.calculate(ySpeed);
            thetaSpeed = thetaSpeedLimiter.calculate(thetaSpeed);
        }

        xSpeed *= Constants.Teleop.PERCENT_LIMIT;
        ySpeed *= Constants.Teleop.PERCENT_LIMIT;
        thetaSpeed *= Constants.Teleop.PERCENT_LIMIT;

        if (slowModeSupplier.getAsBoolean()) {
            xSpeed *= Constants.Teleop.SLOW_MODE_ADJUSTMENT;
            ySpeed *= Constants.Teleop.SLOW_MODE_ADJUSTMENT;
            thetaSpeed *= Constants.Teleop.SLOW_MODE_ADJUSTMENT;
        }

        // the speeds are initially values from -1.0 to 1.0, so we multiply by the max
        // physical velocity to output in m/s.
        drivetrain.drive(
                xSpeed * Constants.Drivetrain.Geometry.MAX_PHYSICAL_VELOCITY_METERS_PER_SECOND,
                ySpeed * Constants.Drivetrain.Geometry.MAX_PHYSICAL_VELOCITY_METERS_PER_SECOND,
                thetaSpeed * Constants.Drivetrain.Geometry.MAX_PHYSICAL_VELOCITY_METERS_PER_SECOND,
                drivetrain.getDriveMode());
    }
}
