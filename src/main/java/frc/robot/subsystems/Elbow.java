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
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;

public class Elbow extends ProfiledPIDSubsystem{
    private CANSparkMax elbowMotor = new CANSparkMax(Constants.Elbow.CANIDs.ELBOW_MOTOR, MotorType.kBrushless);
    private ArmFeedforward feedForward = new ArmFeedforward(Constants.Elbow.FeedForward.kS,
    Constants.Elbow.FeedForward.kG, Constants.Elbow.FeedForward.kV, Constants.Elbow.FeedForward.kA);
    private AnalogEncoder elbowEncoder = new AnalogEncoder(Constants.Elbow.CANIDs.ENCODER_CHANNEL); 
    private DigitalInput bottomhallSensor = new DigitalInput(Constants.Elbow.CANIDs.B_HALL_SENSOR_CHANNEL); 
    private DigitalInput tophallSensor = new DigitalInput(Constants.Elbow.CANIDs.T_HALL_SENSOR_CHANNEL); 
    
    public Elbow() {
        super(new ProfiledPIDController(Constants.Elbow.PIDIDs.kP, Constants.Elbow.PIDIDs.kP,
         Constants.Elbow.PIDIDs.kP, null));
         getController().setTolerance(1.0);
         LoggingManager.getInstance().addGroup("Elbow", new LogGroup(
                    new DeviceLogger<CANSparkMax>(elbowMotor, "Elbow Motor",
                        LogProfileBuilder.buildCANSparkMaxLogItems(elbowMotor))
            ));
        }
    
    
    @Override
    protected void useOutput(double output, State setpoint) {
         elbowMotor.setVoltage(feedForward.calculate(setpoint.position, setpoint.velocity) + output); 
    }
   
    @Override
    protected double getMeasurement() {
        return elbowEncoder.getAbsolutePosition();
        
    }
    protected void setAngle(double angle){
        setGoal(angle);
    }
    protected void tophallTrip(){
        if ((tophallSensor.get() == true) && (elbowMotor.getBusVoltage() > 0)){
            elbowMotor.setVoltage(0);
        }
    }
    protected void bottomhallTrip(){
        if ((bottomhallSensor.get() == true) && (elbowMotor.getBusVoltage() < 0)){
            elbowMotor.setVoltage(0);
        }
    }
}
