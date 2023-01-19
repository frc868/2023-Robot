package frc.robot.commands.auto;

import com.techhounds.houndutil.houndauto.AutoPath;
import com.techhounds.houndutil.houndauto.AutoTrajectoryCommand;

import frc.robot.commands.AutoRoutineHelpers;
import frc.robot.subsystems.Drivetrain;

public class Figure8 extends AutoTrajectoryCommand {
    public Figure8(AutoPath autoPath, Drivetrain drivetrain) {
        super(autoPath);

        addCommands(
                AutoRoutineHelpers.generateSpinningSwervePathFollowingCommand(
                        autoPath.getTrajectories().get(0), drivetrain));
        addRequirements(drivetrain);
    }
}
