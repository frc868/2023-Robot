package frc.robot.commands.auton.paths;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.auton.ShootSequence;
import frc.robot.sensors.Limelight;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * A two ball auto not using trajectories. See the auton sheets to visualize
 * the path.
 */
public class TwoBall extends SequentialCommandGroup {
    public TwoBall(Drivetrain drivetrain, Shooter shooter, Intake intake, Hopper hopper, Limelight limelight) {
        addCommands(
                new InstantCommand(intake::setDown, intake),
                new WaitCommand(1),
                new ParallelRaceGroup(
                        new DriveStraight(0, drivetrain),
                        new RunCommand(intake::runMotors, intake)),
                new ShootSequence(drivetrain, shooter, limelight, hopper),
                new InstantCommand(intake::setUp, intake));

    }
}
