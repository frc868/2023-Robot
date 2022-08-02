package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Constants;

/**
 * Turns to the current gyro angle plus a specified value. Positive is CCW.
 * 
 * @author dr
 */
public class TurnFromCurrentAngle extends ProfiledPIDCommand {
    public TurnFromCurrentAngle(double setpoint, Drivetrain drivetrain) {
        super(new ProfiledPIDController(Constants.Drivetrain.PID.TurnToAngle.kP,
                Constants.Drivetrain.PID.TurnToAngle.kI,
                Constants.Drivetrain.PID.TurnToAngle.kD,
                new TrapezoidProfile.Constraints(Constants.Auton.MAX_ANGULAR_VELOCITY,
                        Constants.Auton.MAX_ANGULAR_ACCELERATION)),
                drivetrain::getGyroAngle, drivetrain.getGyroAngle() + setpoint,
                (output, state) -> drivetrain.drive(0, 0, output, Drivetrain.DriveMode.ROBOT_RELATIVE),
                drivetrain);
        getController().enableContinuousInput(-180, 180);
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }

}
