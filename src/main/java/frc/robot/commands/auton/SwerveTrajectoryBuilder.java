package frc.robot.commands.auton;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class SwerveTrajectoryBuilder {
    /**
     * Creates a SwerveControllerCommand from a given trajectory.
     */
    public static SwerveControllerCommand buildTrajectoryCommand(Trajectory trajectory, Drivetrain drivetrain) {
        return new SwerveControllerCommand(trajectory,
                drivetrain::getPose,
                Constants.Drivetrain.Geometry.KINEMATICS,
                new PIDController(Constants.Drivetrain.PID.Trajectories.X.kP,
                        Constants.Drivetrain.PID.Trajectories.X.kI,
                        Constants.Drivetrain.PID.Trajectories.X.kD),
                new PIDController(Constants.Drivetrain.PID.Trajectories.Y.kP,
                        Constants.Drivetrain.PID.Trajectories.Y.kI,
                        Constants.Drivetrain.PID.Trajectories.Y.kD),
                new ProfiledPIDController(Constants.Drivetrain.PID.Trajectories.Theta.kP,
                        Constants.Drivetrain.PID.Trajectories.Theta.kI,
                        Constants.Drivetrain.PID.Trajectories.Y.kD,
                        new TrapezoidProfile.Constraints(Constants.Auton.MAX_ANGULAR_VELOCITY,
                                Constants.Auton.MAX_ANGULAR_ACCELERATION)),
                drivetrain::setModuleStates,
                drivetrain);
    }
}
