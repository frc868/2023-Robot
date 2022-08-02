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
        super(new ProfiledPIDController(Constants.Drivetrain.PID.TurnToAngle.kP,
                Constants.Drivetrain.PID.TurnToAngle.kI,
                Constants.Drivetrain.PID.TurnToAngle.kD,
                new TrapezoidProfile.Constraints(Constants.Auton.MAX_ANGULAR_VELOCITY,
                        Constants.Auton.MAX_ANGULAR_ACCELERATION)),
                drivetrain::getGyroAngle, setpoint,
                (output, state) -> drivetrain.drive(0, 0, output, Drivetrain.DriveMode.ROBOT_RELATIVE),
                drivetrain);
        getController().enableContinuousInput(-180, 180);
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }

}
