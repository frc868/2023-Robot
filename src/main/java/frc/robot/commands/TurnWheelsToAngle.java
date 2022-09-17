package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;

/**
 * Turn all wheels to a specified angle. This can be used for blocking other
 * robots.
 */
public class TurnWheelsToAngle extends InstantCommand {
    /**
     * Create a TurnWheelsToAngle command.
     * 
     * @param angle      the angle to turn the wheels to, in radians.
     * @param drivetrain the drivetrain.
     */
    public TurnWheelsToAngle(double angle, Drivetrain drivetrain) {
        super(() -> drivetrain.setModuleStates(
                new SwerveModuleState[] {
                        new SwerveModuleState(0, new Rotation2d(angle)),
                        new SwerveModuleState(0, new Rotation2d(angle)),
                        new SwerveModuleState(0, new Rotation2d(angle)),
                        new SwerveModuleState(0, new Rotation2d(angle))
                }), drivetrain);
    }
}
