package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class AutoRoutineHelpers {

    public static Command generateSwervePathFollowingCommand(PathPlannerTrajectory path, Drivetrain drivetrain) {
        return new PPSwerveControllerCommand(
                path,
                drivetrain::getPose,
                Constants.Drivetrain.Geometry.KINEMATICS,
                new PIDController(Constants.Drivetrain.PID.Trajectories.xkP, 0, 0),
                new PIDController(Constants.Drivetrain.PID.Trajectories.ykP, 0, 0),
                new PIDController(Constants.Drivetrain.PID.Trajectories.thetakP, 0, 0),
                (s) -> drivetrain.setModuleStates(s, true, true),
                drivetrain);
    }

    public static Command generateSpinningSwervePathFollowingCommand(PathPlannerTrajectory path,
            Drivetrain drivetrain) {
        return new PPSwerveControllerCommand(
                path,
                drivetrain::getPose,
                Constants.Drivetrain.Geometry.KINEMATICS,
                new PIDController(Constants.Drivetrain.PID.Trajectories.xkP, 0, 0),
                new PIDController(Constants.Drivetrain.PID.Trajectories.ykP, 0, 0),
                new PIDController(0, 0, 0),
                (s) -> {
                    ChassisSpeeds c = Constants.Drivetrain.Geometry.KINEMATICS.toChassisSpeeds(s);
                    c.omegaRadiansPerSecond = Math.PI;

                    drivetrain.setModuleStates(Constants.Drivetrain.Geometry.KINEMATICS.toSwerveModuleStates(c), true,
                            true);
                },
                drivetrain);
    }
}
