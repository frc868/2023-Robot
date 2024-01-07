package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.techhounds.houndutil.houndlib.SparkConfigurator;
import com.techhounds.houndutil.houndlib.subsystems.BaseSingleJointedArm;
import com.techhounds.houndutil.houndlog.interfaces.Log;
import com.techhounds.houndutil.houndlog.interfaces.LoggedObject;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.Constants.Elbow.ElbowPosition;
import static frc.robot.Constants.Elbow.*;

/**
 * The elbow subsystem, controlling the elbow's position.
 * 
 * @author jc
 * @author dr
 */
@LoggedObject
public class Elbow extends SubsystemBase implements BaseSingleJointedArm<ElbowPosition> {
    @Log
    private CANSparkMax motor;
    @Log
    private SparkAbsoluteEncoder encoder;

    private MechanismLigament2d ligament;

    private SingleJointedArmSim armSim = new SingleJointedArmSim(
            MOTOR_GEARBOX_REPR,
            GEARING,
            MOMENT_OF_INERTIA_KG_METERS_SQUARED,
            LENGTH_METERS,
            MIN_ANGLE_RADIANS,
            MAX_ANGLE_RADIANS,
            true,
            0);

    @Log
    private ProfiledPIDController pidController = new ProfiledPIDController(kP, kI, kD, MOVEMENT_CONSTRAINTS);

    @Log
    private ArmFeedforward feedforwardController = new ArmFeedforward(kS, kG, kV, kA);

    @Log(groups = "control")
    private double feedbackVoltage = 0;
    @Log(groups = "control")
    private double feedforwardVoltage = 0;

    private Supplier<Pose3d> elevatorPoseSupplier;

    public Elbow(Supplier<Pose3d> elevatorPoseSupplier, MechanismLigament2d ligament) {
        this.ligament = ligament;
        this.elevatorPoseSupplier = elevatorPoseSupplier;

        motor = SparkConfigurator.createSparkMax(
                MOTOR_ID, MotorType.kBrushless, true,
                (s) -> s.setIdleMode(IdleMode.kBrake),
                (s) -> s.setSmartCurrentLimit(CURRENT_LIMIT),
                (s) -> s.getEncoder().setPositionConversionFactor(ENCODER_ROTATIONS_TO_RADIANS),
                (s) -> s.getEncoder().setVelocityConversionFactor(ENCODER_ROTATIONS_TO_RADIANS / 60.0),
                (s) -> s.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20),
                (s) -> s.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).setInverted(true),
                (s) -> s.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle)
                        .setPositionConversionFactor(ABSOLUTE_ENCODER_ROTATIONS_TO_RADIANS),
                (s) -> s.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle)
                        .setVelocityConversionFactor(ABSOLUTE_ENCODER_ROTATIONS_TO_RADIANS / 60.0),
                (s) -> s.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle)
                        .setZeroOffset(ABSOLUTE_ENCODER_ZERO_OFFSET));

        encoder = motor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

        new Thread(() -> {
            try {
                Thread.sleep(3000);
                motor.getEncoder().setPosition(encoder.getPosition());
            } catch (Exception e) {
                e.printStackTrace(System.err);
            }
        }).start();

        setDefaultCommand(moveToCurrentGoalCommand());
    }

    @Override
    public void periodic() {
        ligament.setAngle(-34 + Units.radiansToDegrees(motor.getEncoder().getPosition()));
    }

    @Override
    public void simulationPeriodic() {
        armSim.setInput(motor.getAppliedOutput());
        armSim.update(0.020);
        motor.getEncoder().setPosition(armSim.getAngleRads());
    }

    @Override
    @Log
    public double getPosition() {
        return motor.getEncoder().getPosition();
    }

    @Log
    public Pose3d getComponentPose() {
        Pose3d elevatorPose = elevatorPoseSupplier.get();
        return new Pose3d(elevatorPose.getTranslation().plus(ELEVATOR_TO_ELBOW), new Rotation3d(0, -getPosition(), 0));
    }

    @Override
    public void resetPosition() {
        motor.getEncoder().setPosition(0);
    }

    /**
     * Sets the voltage of the motor.
     * 
     * @param voltage the voltage, from -12.0v to 12.0v
     */
    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public Command moveToCurrentGoalCommand() {
        return run(() -> {
            feedbackVoltage = pidController.calculate(getPosition());
            feedforwardVoltage = feedforwardController.calculate(pidController.getSetpoint().position,
                    pidController.getSetpoint().velocity);
            setVoltage(feedbackVoltage + feedforwardVoltage);
        }).withName("Move to Current Goal");
    }

    @Override
    public Command moveToPositionCommand(Supplier<ElbowPosition> goalPositionSupplier) {
        return Commands.sequence(
                runOnce(() -> pidController.reset(getPosition())),
                runOnce(() -> pidController.setGoal(goalPositionSupplier.get().value)),
                moveToCurrentGoalCommand().until(pidController::atGoal)).withTimeout(2)
                .withName("Move to Position");
    }

    @Override
    public Command moveToArbitraryPositionCommand(Supplier<Double> goalPositionSupplier) {
        return Commands.sequence(
                runOnce(() -> pidController.reset(getPosition())),
                runOnce(() -> pidController.setGoal(goalPositionSupplier.get())),
                moveToCurrentGoalCommand().until(pidController::atGoal)).withTimeout(2)
                .withName("Move to Arbitrary Position");
    }

    @Override
    public Command movePositionDeltaCommand(Supplier<Double> delta) {
        return Commands.sequence(
                runOnce(() -> pidController.reset(getPosition())),
                runOnce(() -> pidController.setGoal(getPosition() + delta.get())),
                moveToCurrentGoalCommand().until(pidController::atGoal)).withTimeout(2)
                .withName("Move to Arbitrary Position");
    }

    @Override
    public Command holdCurrentPositionCommand() {
        return runOnce(() -> pidController.setGoal(getPosition()))
                .andThen(moveToCurrentGoalCommand());
    }

    @Override
    public Command resetPositionCommand() {
        return runOnce(this::resetPosition);
    }

    @Override
    public Command setOverridenSpeedCommand(Supplier<Double> speed) {
        return run(() -> setVoltage(12.0 * speed.get()))
                .withName("Set Overridden Elbow Speed");
    }

    @Override
    public Command coastMotorsCommand() {
        return runOnce(() -> motor.stopMotor())
                .andThen(() -> motor.setIdleMode(IdleMode.kCoast))
                .finallyDo((d) -> {
                    motor.setIdleMode(IdleMode.kBrake);
                    pidController.reset(getPosition());
                }).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }
}
