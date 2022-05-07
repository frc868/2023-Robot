// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.logging.LogGroup;
import frc.robot.logging.LogProfileBuilder;
import frc.robot.logging.Logger;
import frc.robot.logging.SendableLogger;

/**
 * Drivetrain subsystem, includes all of the motors and the methods with which
 * to drive the bot.
 */
public class Drivetrain extends SubsystemBase {
    private CANSparkMax leftPrimaryMotor = new CANSparkMax(Constants.Drivetrain.CANIDs.L_PRIMARY,
            MotorType.kBrushless);
    private CANSparkMax leftSecondaryMotor = new CANSparkMax(Constants.Drivetrain.CANIDs.L_SECONDARY,
            MotorType.kBrushless);
    private CANSparkMax rightPrimaryMotor = new CANSparkMax(Constants.Drivetrain.CANIDs.R_PRIMARY,
            MotorType.kBrushless);
    private CANSparkMax rightSecondaryMotor = new CANSparkMax(Constants.Drivetrain.CANIDs.R_SECONDARY,
            MotorType.kBrushless);
    private MotorControllerGroup leftMotors = new MotorControllerGroup(leftPrimaryMotor, leftSecondaryMotor);
    private MotorControllerGroup rightMotors = new MotorControllerGroup(rightPrimaryMotor, rightSecondaryMotor);
    private DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);
    private AHRS navx = new AHRS(SerialPort.Port.kMXP);

    private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.Drivetrain.TRACK_WIDTH);
    private DifferentialDriveOdometry odometry;
    private Field2d field = new Field2d();

    private LogGroup logger = new LogGroup("Drivetrain",
            new Logger<?>[] {
                    new Logger<CANSparkMax>(leftPrimaryMotor, "Left Primary Motor",
                            LogProfileBuilder.buildCANSparkMaxLogItems(leftPrimaryMotor)),
                    new Logger<CANSparkMax>(leftSecondaryMotor, "Left Secondary Motor",
                            LogProfileBuilder.buildCANSparkMaxLogItems(leftSecondaryMotor)),
                    new Logger<CANSparkMax>(rightPrimaryMotor, "Right Primary Motor",
                            LogProfileBuilder.buildCANSparkMaxLogItems(rightPrimaryMotor)),
                    new Logger<CANSparkMax>(rightSecondaryMotor, "Right Secondary Motor",
                            LogProfileBuilder.buildCANSparkMaxLogItems(rightSecondaryMotor)),
                    new Logger<AHRS>(navx, "NavX",
                            LogProfileBuilder.buildNavXLogItems(navx)),
                    new SendableLogger("field", field),
            });

    /**
     * Initializes the drivetrain.
     */
    public Drivetrain() {
        leftPrimaryMotor.getEncoder().setPositionConversionFactor(Constants.Drivetrain.ENCODER_DISTANCE_PER_PULSE);
        rightPrimaryMotor.getEncoder().setPositionConversionFactor(Constants.Drivetrain.ENCODER_DISTANCE_PER_PULSE);
        leftMotors.setInverted(Constants.Drivetrain.IS_LEFT_INVERTED);
        rightMotors.setInverted(Constants.Drivetrain.IS_RIGHT_INVERTED);
        drive.setMaxOutput(0.8);
        odometry = new DifferentialDriveOdometry(navx.getRotation2d());
        logger.init();
    }

    /**
     * Runs every 20ms. In this method, all we do is run SmartDashboard/logging
     * related functions (do NOT run any code that should belong in a command here!)
     */
    @Override
    public void periodic() {
        logger.run();
        odometry.update(Rotation2d.fromDegrees(-getGyroAngle()), getLeftPosition(), getRightPosition());
        field.setRobotPose(odometry.getPoseMeters());
    }

    /**
     * Gets the pose of the robot (the estimated location on the field).
     * 
     * @return the pose
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Gets the current wheel speeds of the robot.
     * 
     * @return the current wheel speeds
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftPrimaryMotor.getEncoder().getVelocity(),
                rightPrimaryMotor.getEncoder().getVelocity());
    }

    /**
     * Gets the defined kinematics for the drivetrain.
     */
    public DifferentialDriveKinematics getKinematics() {
        return kinematics;
    }

    /**
     * Resets the odometry of the robot to a specified pose.
     * 
     * @param pose the pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(pose, navx.getRotation2d());
    }

    /**
     * Runs the drivetrain in arcade mode.
     * 
     * @param speed the speed from -1 to 1 with which to drive the motors (usually
     *              taken from the L stick Y axis of a controller)
     * @param rot   the rotation from -1 to 1 with which to rotate the robot
     *              (usually taken from the R stick X axis of a controller)
     */
    public void arcadeDrive(double speed, double rot) {
        drive.arcadeDrive(speed, rot);
    }

    /**
     * Runs the drivetrain in tank mode.
     * 
     * @param leftSpeed  the speed from -1 to 1 with which to run the left motors
     *                   (usually taken from the L stick Y axis of a controller)
     * @param rightSpeed the speed from -1 to 1 with which to run the right motors
     *                   (usually taken from the R stick Y axis of a controller)
     */
    public void tankDrive(double leftSpeed, double rightSpeed) {
        drive.tankDrive(leftSpeed, rightSpeed);
    }

    /**
     * Control the motors with with volts (useful for feed-forward calculations).
     * 
     * @param leftVolts  the left output
     * @param rightVolts the right output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftMotors.setVoltage(leftVolts);
        rightMotors.setVoltage(rightVolts);
        drive.feed();
    }

    /**
     * Resets the drivetrain encoders by setting their positions to zero.
     */
    public void resetEncoders() {
        leftPrimaryMotor.getEncoder().setPosition(0);
        rightPrimaryMotor.getEncoder().setPosition(0);
    }

    /**
     * Gets the position of the left primary motor encoder.
     * 
     * @return the position of the left primary motor encoder
     */
    public double getLeftPosition() {
        return leftPrimaryMotor.getEncoder().getPosition();
    }

    /**
     * Gets the position of the right primary motor encoder.
     * 
     * @return the position of the right primary motor encoder
     */

    public double getRightPosition() {
        return rightPrimaryMotor.getEncoder().getPosition();
    }

    /**
     * Gets the average position of the left and right encoders.
     * 
     * @return the average position of the left and right encoders.
     */
    public double getPosition() {
        return (double) (leftPrimaryMotor.getEncoder().getPosition() + rightPrimaryMotor.getEncoder().getPosition())
                / 2.0;
    }

    /**
     * Sets the max output of the drivetrain.
     * 
     * @param maxOutput the maximum output percentage (0 to 1) of the drivetrain
     */
    public void setMaxOutput(double maxOutput) {
        drive.setMaxOutput(maxOutput);
    }

    /**
     * Stops the drivetrain.
     */
    public void stop() {
        tankDrive(0, 0);
    }

    /**
     * Gets the current angle of the gyro.
     */
    public double getGyroAngle() {
        return navx.getAngle();
    }

    /**
     * Reset the navX angle.
     */
    public void resetGyro() {
        navx.reset();
    }
}
