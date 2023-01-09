package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.techhounds.houndutil.houndlib.auto.AutoManager;
import com.techhounds.houndutil.houndlib.auto.PPAutoPath;
import com.techhounds.houndutil.houndlib.auto.PPAutoTrajectoryCommand;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoRoutines;
import frc.robot.subsystems.Drivetrain;

public class Circle extends SequentialCommandGroup implements PPAutoTrajectoryCommand {
    private PPAutoPath autoPath;

    public Circle(PPAutoPath autoPath, Drivetrain drivetrain) {
        this.autoPath = autoPath;

        PathPlannerTrajectory path = autoPath.getTrajectories().get(0);
        addCommands(
                new FollowPathWithEvents(
                        AutoRoutines.generateSwervePathFollowingCommand(path, drivetrain),
                        path.getMarkers(),
                        AutoManager.getInstance().getEventMap()));
        addRequirements(drivetrain);
    }

    @Override
    public PPAutoPath getAutoPath() {
        return autoPath;
    }
}
