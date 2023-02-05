package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.techhounds.houndutil.houndauto.AutoManager;
import com.techhounds.houndutil.houndauto.AutoPath;
import com.techhounds.houndutil.houndauto.AutoTrajectoryCommand;

import frc.robot.subsystems.Drivetrain;

public class Autos {
    /**
     * Creates the circle trajectory command.
     * 
     * @param autoPath   the {@link AutoPath} containing the circle trajectory.
     * @param drivetrain the drivetrain
     * @return the command
     */
    public static AutoTrajectoryCommand circle(AutoPath autoPath, Drivetrain drivetrain) {
        PathPlannerTrajectory path = autoPath.getTrajectories().get(0);
        return new AutoTrajectoryCommand(autoPath, new FollowPathWithEvents(
                drivetrain.pathFollowingCommand(path),
                path.getMarkers(),
                AutoManager.getInstance().getEventMap()));
    }

    /**
     * Creates the Figure 8 trajectory command.
     * 
     * @param autoPath   the {@link AutoPath} containing the Figure 8 trajectory.
     * @param drivetrain the drivetrain
     * @return the command
     */
    public static AutoTrajectoryCommand figure8(AutoPath autoPath, Drivetrain drivetrain) {
        return new AutoTrajectoryCommand(autoPath, drivetrain.spinningPathFollowingCommand(
                autoPath.getTrajectories().get(0)));
    }
}
