package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Constants;

/**
 * Turns to a specified gyro angle. This will be relative to the field, and
 * counterclockwise (a value of 90 will make the robot face the left side).
 * 
 * @author dr
 */
public class TurnToAngle extends ProfiledPIDCommand {
    public TurnToAngle(double setpoint, Drivetrain drivetrain) {
        super(new ProfiledPIDController(Constants.Drivetrain.PIDConstants.TurnToAngle.kP,
                Constants.Drivetrain.PIDConstants.TurnToAngle.kI,
                Constants.Drivetrain.PIDConstants.TurnToAngle.kD,
                new TrapezoidProfile.Constraints(Constants.Drivetrain.MAX_VELOCITY,
                        Constants.Drivetrain.MAX_ACCELERATION)),
                drivetrain::getGyroAngle, setpoint,
                (output, state) -> drivetrain.drive(0, 0, output, false),
                drivetrain);
        getController().enableContinuousInput(-180, 180);
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }

}
