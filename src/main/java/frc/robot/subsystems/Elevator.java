package frc.robot.subsystems;

import java.util.Map;
import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.techhounds.houndutil.houndlib.SparkMaxConfigurator;
import com.techhounds.houndutil.houndlib.Utils;
import com.techhounds.houndutil.houndlib.subsystems.BaseElevator;
import com.techhounds.houndutil.houndlog.interfaces.Log;
import com.techhounds.houndutil.houndlog.interfaces.LoggedObject;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Overrides;
import frc.robot.Modes;
import frc.robot.Constants.Elevator.ElevatorPosition;
import frc.robot.GamePieceLocation.GamePiece;
import frc.robot.GamePieceLocation.Level;
import static frc.robot.Constants.Elevator.*;

/**
 * The elevator subsystem, with two motors and motion profling.
 * 
 * @author hr
 * @author jt
 */
@LoggedObject
public class Elevator extends SubsystemBase implements BaseElevator<ElevatorPosition> {
    /** The left motor of the elevator. */
    @Log
    private CANSparkMax leftMotor;

    /** The right motor of the elevator. */
    @Log
    private CANSparkMax rightMotor;

    /** Hall effect sensor that represents the minimum extension of the elevator. */
    @Log
    private DigitalInput bottomHallEffect = new DigitalInput(BOTTOM_HALL_SENSOR_PORT);

    @Log
    private ProfiledPIDController pidController = new ProfiledPIDController(kP, kI, kD, NORMAL_MOVEMENT_CONSTRAINTS);

    /**
     * The feedforward controller (calculates voltage based on the setpoint's
     * velocity and acceleration).
     */
    @Log
    private ElevatorFeedforward feedforwardController = new ElevatorFeedforward(kS, kG, kV, kA);

    /** The ligament of the complete mechanism body that this subsystem controls. */
    private MechanismLigament2d ligament;

    /**
     * The physics simulator for this mechanism. The elevator sim takes in voltages
     * from the motor and simulates an elevator under the influence of gravity.
     */
    private ElevatorSim elevatorSim = new ElevatorSim(
            MOTOR_GEARBOX_REPR,
            GEARING,
            CARRIAGE_MASS_KG,
            DRUM_RADIUS_METERS,
            MIN_HEIGHT_METERS,
            MAX_HEIGHT_METERS,
            true,
            0);

    /**
     * The wrapper of the bottomHallEffect for simulation. This allows you to set
     * the values of the sensor in sim without changing the values IRL. This value
     * is not used when the robot is running IRL.
     */
    private DIOSim bottomHallEffectSim;

    @Log(groups = "control")
    private double feedbackVoltage = 0;
    @Log(groups = "control")
    private double feedforwardVoltage = 0;

    /**
     * Initializes the elevator.
     */
    public Elevator(MechanismLigament2d ligament) {

        leftMotor = SparkMaxConfigurator.create(
                LEFT_MOTOR_ID, MotorType.kBrushless, false,
                (s) -> s.setIdleMode(IdleMode.kBrake),
                (s) -> s.setSmartCurrentLimit(CURRENT_LIMIT),
                (s) -> s.getEncoder().setPositionConversionFactor(ENCODER_ROTATIONS_TO_METERS),
                (s) -> s.getEncoder().setVelocityConversionFactor(ENCODER_ROTATIONS_TO_METERS / 60.0));

        rightMotor = SparkMaxConfigurator.create(
                RIGHT_MOTOR_ID, MotorType.kBrushless, true,
                (s) -> s.setIdleMode(IdleMode.kBrake),
                (s) -> s.setSmartCurrentLimit(CURRENT_LIMIT),
                (s) -> s.getEncoder().setPositionConversionFactor(ENCODER_ROTATIONS_TO_METERS),
                (s) -> s.getEncoder().setVelocityConversionFactor(ENCODER_ROTATIONS_TO_METERS / 60.0),
                (s) -> s.follow(leftMotor, true));

        pidController.setTolerance(TOLERANCE);

        this.ligament = ligament;

        if (RobotBase.isSimulation()) {
            bottomHallEffectSim = new DIOSim(bottomHallEffect);
            bottomHallEffectSim.setValue(false);
        }

        new Trigger(bottomHallEffect::get).whileTrue(
                Commands.parallel(
                        Commands.runOnce(this::resetPosition).ignoringDisable(true),
                        Commands.runOnce(Modes::enableInitialized)).ignoringDisable(true));

        setDefaultCommand(moveToCurrentGoalCommand());
    }

    /**
     * Runs every 20ms. This updates the ligament sent over NetworkTables.
     */
    @Override
    public void periodic() {
        ligament.setLength(-0.71 + getPosition());
    }

    /**
     * Updates simulation-specific variables.
     */
    @Override
    public void simulationPeriodic() {
        // set the input (the voltage of the motor)
        elevatorSim.setInput(leftMotor.getAppliedOutput());
        // update the sim
        elevatorSim.update(0.020);
        leftMotor.getEncoder().setPosition(elevatorSim.getPositionMeters());
        bottomHallEffectSim.setValue(elevatorSim.getPositionMeters() <= 0);
    }

    @Override
    @Log
    public double getPosition() {
        return leftMotor.getEncoder().getPosition();
    }

    @Log
    public Pose3d getMiddleSegmentPose() {
        double lateralPosition = getPosition() >= STARTING_MIDDLE_SEGMENT_POSITION
                ? getPosition() - STARTING_MIDDLE_SEGMENT_POSITION
                : 0;
        return new Pose3d(BASE_MIDDLE_SEGMENT_POSE.getTranslation().plus(new Translation3d(
                Math.sin(ANGLE) * lateralPosition, 0, Math.cos(ANGLE) * lateralPosition)),
                BASE_MIDDLE_SEGMENT_POSE.getRotation());
    }

    @Log
    public Pose3d getInnerSegmentPose() {
        double lateralPosition = getPosition() >= STARTING_INNER_SEGMENT_POSITION
                ? getPosition() - STARTING_INNER_SEGMENT_POSITION
                : 0;
        return new Pose3d(BASE_INNER_SEGMENT_POSE.getTranslation().plus(new Translation3d(
                Math.sin(ANGLE) * lateralPosition, 0, Math.cos(ANGLE) * lateralPosition)),
                BASE_INNER_SEGMENT_POSE.getRotation());

    }

    @Log
    public Pose3d getCarriagePose() {
        return new Pose3d(BASE_CARRIAGE_POSE.getTranslation().plus(new Translation3d(
                Math.sin(ANGLE) * getPosition(), 0, Math.cos(ANGLE) * getPosition())),
                BASE_CARRIAGE_POSE.getRotation());
    }

    @Override
    public void resetPosition() {
        leftMotor.getEncoder().setPosition(0);
        rightMotor.getEncoder().setPosition(0);
    }

    @Override
    public void setVoltage(double voltage) {
        if (!Utils.limitMechanism(bottomHallEffect.get(), false,
                voltage) || Overrides.MECH_LIMITS_DISABLE.getStatus())
            leftMotor.setVoltage(voltage);
        else {
            leftMotor.setVoltage(0);
        }
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
    public Command moveToPositionCommand(Supplier<ElevatorPosition> goalPositionSupplier) {
        return Commands.sequence(
                runOnce(() -> {
                    if (goalPositionSupplier.get() == ElevatorPosition.BOTTOM)
                        pidController.setConstraints(STOWING_MOVEMENT_CONSTRAINTS);
                    else
                        pidController.setConstraints(NORMAL_MOVEMENT_CONSTRAINTS);
                }),
                runOnce(() -> pidController.reset(getPosition())),
                runOnce(() -> pidController.setGoal(goalPositionSupplier.get().value)), // also sets 0 velocity
                moveToCurrentGoalCommand().until(pidController::atGoal))
                .withTimeout(2)
                .finallyDo((d) -> pidController.setConstraints(NORMAL_MOVEMENT_CONSTRAINTS));
    }

    @Override
    public Command moveToArbitraryPositionCommand(Supplier<Double> goalPositionSupplier) {
        return Commands.sequence(
                runOnce(() -> pidController.reset(getPosition())),
                runOnce(() -> pidController.setGoal(goalPositionSupplier.get())),
                moveToCurrentGoalCommand().until(pidController::atGoal))
                .withTimeout(2);
    }

    @Override
    public Command movePositionDeltaCommand(Supplier<Double> delta) {
        return moveToArbitraryPositionCommand(() -> pidController.getGoal().position + delta.get());
    }

    @Override
    public Command holdCurrentPositionCommand() {
        return runOnce(() -> pidController.setGoal(getPosition())).andThen(moveToCurrentGoalCommand());
    }

    @Override
    public Command resetPositionCommand() {
        return runOnce(this::resetPosition);
    }

    @Override
    public Command setOverridenSpeedCommand(Supplier<Double> speed) {
        return run(() -> setVoltage(12.0 * speed.get()))
                .withName("Set Overridden Elevator Speed");
    }

    @Override
    public Command coastMotorsCommand() {
        return runOnce(() -> leftMotor.stopMotor())
                .andThen(() -> {
                    leftMotor.setIdleMode(IdleMode.kCoast);
                    rightMotor.setIdleMode(IdleMode.kCoast);
                })
                .finallyDo((d) -> {
                    leftMotor.setIdleMode(IdleMode.kBrake);
                    rightMotor.setIdleMode(IdleMode.kBrake);
                    pidController.reset(getPosition());
                }).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    public Command moveToScoringPositionCommand(Supplier<GamePiece> gamePieceMode,
            Supplier<Level> scoringMode, Intake intake, Elbow elbow) {
        return Commands.select(
                Map.of(
                        GamePiece.CONE,
                        Commands.select(
                                Map.of(
                                        Level.HYBRID,
                                        moveToPositionCommand(() -> ElevatorPosition.CONE_LOW),
                                        Level.MIDDLE,
                                        moveToPositionCommand(() -> ElevatorPosition.CONE_MID),
                                        Level.HIGH,
                                        moveToPositionCommand(() -> ElevatorPosition.CONE_HIGH)),
                                scoringMode::get),
                        GamePiece.CUBE,
                        Commands.select(
                                Map.of(
                                        Level.HYBRID,
                                        moveToPositionCommand(() -> ElevatorPosition.CUBE_LOW),
                                        Level.MIDDLE,
                                        moveToPositionCommand(() -> ElevatorPosition.CUBE_MID),
                                        Level.HIGH,
                                        moveToPositionCommand(() -> ElevatorPosition.CUBE_HIGH)),
                                scoringMode::get)),
                gamePieceMode::get);
    }

    public Command autoHallResetPositionCommand() {
        return startEnd(() -> setVoltage(-1.5), () -> {
            this.resetPosition();
            Modes.enableInitialized();
            leftMotor.stopMotor();
        }).until(bottomHallEffect::get).withName("Zero Encoder");
    }
}
