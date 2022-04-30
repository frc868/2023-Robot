package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

/**
 * Drives the drivetrain using a RAMSETE controller. Accepts a trajectory to
 * follow as input.
 * 
 * @author dr
 */
public class DrivetrainRamsete extends RamseteCommand {
    public DrivetrainRamsete(Trajectory trajectory, Drivetrain drivetrain) {
        super(
                trajectory,
                drivetrain::getPose,
                new RamseteController(),
                new SimpleMotorFeedforward(Constants.Drivetrain.kS, Constants.Drivetrain.kV, Constants.Drivetrain.kA),
                drivetrain.getKinematics(),
                drivetrain::getWheelSpeeds,
                new PIDController(Constants.Drivetrain.kP, 0, 0),
                new PIDController(Constants.Drivetrain.kP, 0, 0),
                drivetrain::tankDriveVolts,
                drivetrain);
    }
}
