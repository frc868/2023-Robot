package frc.robot.commands.auton.paths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveStraight;
import frc.robot.sensors.Limelight;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * A zero ball auto that leaves the tarmac, not using trajectories. See the
 * auton sheets to visualize the path.
 */
public class LeaveTarmac extends SequentialCommandGroup {
    public LeaveTarmac(Drivetrain drivetrain, Shooter shooter, Intake intake, Hopper hopper, Limelight limelight) {
        addCommands(
                new DriveStraight(0, drivetrain));
    }
}
