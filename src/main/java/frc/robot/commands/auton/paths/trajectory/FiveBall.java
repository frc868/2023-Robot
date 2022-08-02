package frc.robot.commands.auton.paths.trajectory;

import java.util.HashMap;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auton.ShootSequence;
import frc.robot.commands.auton.SwerveTrajectoryBuilder;
import frc.robot.sensors.Astra;
import frc.robot.sensors.Limelight;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * A five ball auto using trajectories. See the auton sheets to visualize
 * the path.
 */
public class FiveBall extends SequentialCommandGroup {
    public FiveBall(HashMap<String, Trajectory> trajectories, Drivetrain drivetrain, Shooter shooter, Intake intake,
            Hopper hopper, Limelight limelight,
            Astra astra) {
        addCommands(
                new InstantCommand(intake::setDown, intake), // intake down
                new ShootSequence(drivetrain, shooter, limelight, hopper).withTimeout(4), // shoot 1st ball
                new ParallelRaceGroup( // drive and intake 2nd ball
                        SwerveTrajectoryBuilder.buildTrajectoryCommand(trajectories.get("5Ball.To2and3"), drivetrain),
                        new RunCommand(intake::runMotors, intake)),
                new ShootSequence(drivetrain, shooter, limelight, hopper).withTimeout(4), // shoot 2nd and 3rd ball
                new ParallelRaceGroup(
                        SwerveTrajectoryBuilder.buildTrajectoryCommand(trajectories.get("5Ball.To4and5"), drivetrain),
                        new RunCommand(intake::runMotors, intake)), // drive and intake 4th and 5th ball
                new ParallelRaceGroup(
                        new WaitCommand(3),
                        new RunCommand(intake::runMotors, intake)), // wait for human player to give 5th ball
                SwerveTrajectoryBuilder.buildTrajectoryCommand(trajectories.get("5Ball.ToGoal"), drivetrain),
                new ShootSequence(drivetrain, shooter, limelight, hopper).withTimeout(4), // shoot 4th and 5th ball
                new InstantCommand(intake::setUp, intake)); // intake up
    }
}
