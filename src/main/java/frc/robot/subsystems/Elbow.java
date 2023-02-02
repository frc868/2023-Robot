package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.techhounds.houndutil.houndlog.LogGroup;
import com.techhounds.houndutil.houndlog.LogProfileBuilder;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.loggers.DeviceLogger;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
/**
 * Elbow subsystem, one CAN Motor and encoder, one feedforward object, and two hall effect sensors.         
 */
public class Elbow extends ProfiledPIDSubsystem{
    /**
     * Motor that controls the elbow.
     */
    private CANSparkMax elbowMotor = new CANSparkMax(Constants.Elbow.CANIDs.ELBOW_MOTOR, MotorType.kBrushless);
    /**
     * Feedforward object for calculating future disturbances 
     */
    private ArmFeedforward feedForward = new ArmFeedforward(Constants.Elbow.FeedForward.kS,
        Constants.Elbow.FeedForward.kG, Constants.Elbow.FeedForward.kV, Constants.Elbow.FeedForward.kA);
    /**
     * Encoder that returns position in the motor.
     */
    private AnalogEncoder elbowEncoder = new AnalogEncoder(Constants.Elbow.CANIDs.ENCODER_CHANNEL); 
    /**
     * Hall effect sensors in the motor that will limit freedom of movement.    
     */
    private DigitalInput bottomhallEffect = new DigitalInput(Constants.Elbow.CANIDs.B_HALL_SENSOR_CHANNEL); 
    private DigitalInput tophallEffect = new DigitalInput(Constants.Elbow.CANIDs.T_HALL_SENSOR_CHANNEL); 
    /**
     * Constructs profiled pid controller and motor logger objects.
     * Resets arm angle.
     */
    public Elbow() {
        super(new ProfiledPIDController(Constants.Elbow.PID.kP, Constants.Elbow.PID.kP,
         Constants.Elbow.PID.kP, new TrapezoidProfile.Constraints(Constants.Elbow.PID.MAX_VELOCITY,
         Constants.Elbow.PID.MAX_ACCELERATION)));
         getController().setTolerance(1.0);
         LoggingManager.getInstance().addGroup("Elbow", new LogGroup(
                    new DeviceLogger<CANSparkMax>(elbowMotor, "Elbow Motor",
                        LogProfileBuilder.buildCANSparkMaxLogItems(elbowMotor))
            ));
        setGoal(Constants.Elbow.ArmStates.INITIAL_POSITION);
        }
    /**
     * Gets the output of pid controller and calculates setpoint using feedforward.
     * New voltage will be calculated setpoint + output of pid.
     * @param output from pid controller and desired position.
     */
    
    @Override
    protected void useOutput(double output, State setpoint) {
         elbowMotor.setVoltage(feedForward.calculate(setpoint.position, setpoint.velocity) + output); 
    }
    /**
     * Returns absolute angle measurement of encoder. 
     */
   
    @Override
    protected double getMeasurement() {
        return elbowEncoder.getAbsolutePosition();
    }
    /**
     * Checks if top hall effect is triggered and stops elbow from going up.
     */
    protected void tophallTrip(){
        if ((tophallEffect.get() == true) && (elbowMotor.getBusVoltage() > 0)){
            elbowMotor.setVoltage(0);
        }
    }
    /**
     * Checks if bottom hall effect is triggered and stops elbow from going down.
     */
    protected void bottomhallTrip(){
        if ((bottomhallEffect.get() == true) && (elbowMotor.getBusVoltage() < 0)){
            elbowMotor.setVoltage(0);
        }
    }
     /**
     * Sets angle to desired angle.
     * @param angle sets motor to specified angle.
     */
    protected void SetDesiredPositionCommand(double angle){
        setGoal(angle);
    }
    /**
     * Scoring position for cone, hole above the pole.
     */
    protected Command SetScoringCommand(){
        return new RunCommand(()-> {
            SetDesiredPositionCommand(Constants.Elbow.ArmStates.SCORING_POSITION);
        });
    }
    /**
     * Elbow pointing manipulator straight forward.
     */
    protected Command SetGamePieceCollectCommand(){
        return new RunCommand(()-> {
            SetDesiredPositionCommand(Constants.Elbow.ArmStates.COLLECT_GAME_PIECE_POSITION);
        });
    }
    /**
     * Stops the motor.
     */
    protected void Stop(){
        elbowMotor.setVoltage(0);
    }
}
