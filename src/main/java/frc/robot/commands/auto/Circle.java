package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.techhounds.houndutil.houndlib.auto.AutoManager;
import com.techhounds.houndutil.houndlib.auto.AutoPath;
import com.techhounds.houndutil.houndlib.auto.AutoTrajectoryCommand;

import frc.robot.commands.AutoRoutineHelpers;
import frc.robot.subsystems.Drivetrain;

public class Circle extends AutoTrajectoryCommand {
    public Circle(AutoPath autoPath, Drivetrain drivetrain) {
        super(autoPath);

        PathPlannerTrajectory path = autoPath.getTrajectories().get(0);
        addCommands(
                new FollowPathWithEvents(
                        AutoRoutineHelpers.generateSwervePathFollowingCommand(path, drivetrain),
                        path.getMarkers(),
                        AutoManager.getInstance().getEventMap()));
    }
}
