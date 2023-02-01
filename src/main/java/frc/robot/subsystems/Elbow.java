package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;

public class Elbow extends ProfiledPIDSubsystem{
    private CANSparkMax elbowMotor = new CANSparkMax(Constants.Elbow.CANIDs.ELBOW_MOTOR, MotorType.kBrushless);
    private ArmFeedforward feedfoward = new ArmFeedforward(Constants.Elbow.FeedForward.kS,
    Constants.Elbow.FeedForward.kG, Constants.Elbow.FeedForward.kV, Constants.Elbow.FeedForward.kA);
    private AnalogEncoder elbowEncoder = new AnalogEncoder(0); //untested channel

    public Elbow(ProfiledPIDController controller) {
        super(new ProfiledPIDController(Constants.Elbow.PIDIDs.kP, Constants.Elbow.PIDIDs.kP,
         Constants.Elbow.PIDIDs.kP, null));
         getController().setTolerance(1.0);
        }

    @Override
    protected void useOutput(double output, State setpoint) {
        // elbowMotor.setVoltage(output + feedforward.calculate(getMeasurement(), setpoint)); 
    }
   
     

    @Override
    protected double getMeasurement() {
        return elbowEncoder.getAbsolutePosition();
        
    }
    
}
