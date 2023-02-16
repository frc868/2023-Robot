package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.DriveMode;

/**
 * Turns the robot to an angle based on gyro
 * shamelessly stolen from the 2022 B3 repo
 * 
 * @author jt
 */
public class TurnToAngle extends PIDCommand {
    private Drivetrain drivetrain;

    public TurnToAngle(double setpoint, Drivetrain drivetrain) {
        super(new PIDController(Constants.Gains.DriveMotors.kP.get(),
                Constants.Gains.DriveMotors.kI.get(),
                Constants.Gains.DriveMotors.kD.get()),
                () -> drivetrain.getGyroAngle(),
                () -> setpoint,
                d -> drivetrain.drive(0, 0, d, DriveMode.FIELD_ORIENTED),
                drivetrain);
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        this.drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }

}
