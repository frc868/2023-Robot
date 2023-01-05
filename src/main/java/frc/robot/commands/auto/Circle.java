package frc.robot.commands.auto;

import java.util.HashMap;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.techhounds.houndutil.houndlib.auto.PPAutoPath;
import com.techhounds.houndutil.houndlib.auto.PPAutoTrajectoryCommand;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoRoutines;
import frc.robot.subsystems.Drivetrain;

public class Circle extends SequentialCommandGroup implements PPAutoTrajectoryCommand {
    private PPAutoPath autoPath;

    public Circle(PPAutoPath autoPath, Drivetrain drivetrain) {
        this.autoPath = autoPath;

        PathPlannerTrajectory path = autoPath.getTrajectories().get(0);
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("event1", new PrintCommand("Marker 1"));
        eventMap.put("event2", new PrintCommand("Marker 2"));
        eventMap.put("event3", new PrintCommand("Marker 3"));
        eventMap.put("event4", new PrintCommand("Marker 4"));
        addCommands(
                new FollowPathWithEvents(
                        AutoRoutines.generateSwervePathFollowingCommand(path, drivetrain),
                        path.getMarkers(),
                        eventMap));
        addRequirements(drivetrain);
    }

    @Override
    public PPAutoPath getAutoPath() {
        return autoPath;
    }
}
