package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.RunShooterLockedSpeed;
import frc.robot.commands.TurnToGoal;
import frc.robot.sensors.Limelight;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;

/**
 * Defines the sequence of steps we use to shoot a ball. Could be mapped to a
 * controller button to automate the shooting sequence. TODO: test this.
 * 
 * @author dr
 */
public class ShootSequence extends SequentialCommandGroup {
    public ShootSequence(Drivetrain drivetrain, Shooter shooter, Limelight limelight, Hopper hopper) {
        addCommands(
                new InstantCommand(limelight::setShootingMode),
                new TurnToGoal(drivetrain, limelight).withTimeout(1.0),
                new ParallelRaceGroup(
                        new RunShooterLockedSpeed(shooter, limelight),
                        new SequentialCommandGroup(
                                new InstantCommand(hopper::gatekeepersIn, hopper),
                                new RunCommand(hopper::runMotor, hopper).withTimeout(1.5),
                                new InstantCommand(hopper::gatekeepersOut, hopper))),
                new InstantCommand(limelight::setDriverAssistMode));

    }
}
