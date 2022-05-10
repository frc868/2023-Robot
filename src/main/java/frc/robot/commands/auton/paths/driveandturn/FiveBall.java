package frc.robot.commands.auton.paths.driveandturn;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.TurnToAngleGyro;
import frc.robot.commands.TurnToBall;
import frc.robot.commands.auton.ShootSequence;
import frc.robot.sensors.Astra;
import frc.robot.sensors.Limelight;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * A five ball auto not using trajectories. See the auton sheets to visualize
 * the path.
 */
public class FiveBall extends SequentialCommandGroup {
    public FiveBall(Drivetrain drivetrain, Shooter shooter, Intake intake, Hopper hopper, Limelight limelight,
            Astra astra) {
        addCommands(
                new InstantCommand(intake::setDown, intake), // intake down
                new WaitCommand(0.5), // wait for intake to come down
                new ParallelRaceGroup( // drive and intake 2nd ball
                        new DriveStraight(0, drivetrain),
                        new RunCommand(intake::runMotors, intake)),
                new ShootSequence(drivetrain, shooter, limelight, hopper).withTimeout(4), // shoot 1st and 2nd ball
                new TurnToAngleGyro(150, drivetrain).withTimeout(2), // turn to general location of 3rd ball to
                                                                     // put in camera FOV
                new TurnToBall(drivetrain, astra).withTimeout(1), // turn to ball precisely with camera
                new ParallelRaceGroup(
                        new DriveStraight(0, drivetrain),
                        new RunCommand(intake::runMotors, intake)), // drive and intake 3rd ball
                new TurnToAngleGyro(-25, drivetrain), // turn to goal enough to get in limelight frame
                new ShootSequence(drivetrain, shooter, limelight, hopper).withTimeout(4), // shoot 3rd ball
                new TurnToBall(drivetrain, astra).withTimeout(1), // turn to 4th ball
                new ParallelRaceGroup(
                        new DriveStraight(0, drivetrain),
                        new RunCommand(intake::runMotors, intake)), // drive and intake 4th ball
                new ParallelRaceGroup(
                        new WaitCommand(3),
                        new RunCommand(intake::runMotors, intake)), // wait for human player to give 5th ball
                new DriveStraight(-0, drivetrain), // drive back to goal
                new ShootSequence(drivetrain, shooter, limelight, hopper).withTimeout(4), // shoot 4th and 5th ball
                new InstantCommand(intake::setUp, intake)); // intake up
    }
}
