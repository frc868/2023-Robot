package frc.robot.commands.auto;

import com.techhounds.houndutil.houndlib.auto.PPAutoPath;
import com.techhounds.houndutil.houndlib.auto.PPAutoTrajectoryCommand;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoRoutines;
import frc.robot.subsystems.Drivetrain;

public class Figure8 extends SequentialCommandGroup implements PPAutoTrajectoryCommand {
    private PPAutoPath autoPath;

    public Figure8(PPAutoPath autoPath, Drivetrain drivetrain) {
        this.autoPath = autoPath;

        addCommands(
                AutoRoutines.generateSpinningSwervePathFollowingCommand(autoPath.getTrajectories().get(0), drivetrain));
        addRequirements(drivetrain);
    }

    @Override
    public PPAutoPath getAutoPath() {
        return autoPath;
    }
}
