// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;

/**
 * Drives a specified distance (in m) straight ahead, using a PID loop.
 */
public class DriveStraight extends ProfiledPIDCommand {
    private final Drivetrain drivetrain;

    /**
     * Create a new DriveStraight command.
     *
     * @param distance The distance to drive
     */
    public DriveStraight(double distance, Drivetrain drivetrain) {
        super(new ProfiledPIDController(Constants.Drivetrain.PID.TurnToAngle.kP,
                Constants.Drivetrain.PID.TurnToAngle.kI,
                Constants.Drivetrain.PID.TurnToAngle.kD,
                new TrapezoidProfile.Constraints(Constants.Auton.MAX_VELOCITY,
                        Constants.Auton.MAX_ACCELERATION)),
                drivetrain::getDriveEncoderPosition, distance,
                (output, state) -> drivetrain.drive(output, 0, 0, Drivetrain.DriveMode.ROBOT_RELATIVE),
                drivetrain);

        this.drivetrain = drivetrain;
        addRequirements(drivetrain);

        getController().setTolerance(0.01);
    }

    @Override
    public void initialize() {
        drivetrain.resetDriveEncoders();
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }
}
