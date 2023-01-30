package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class Elbow extends PIDSubsystem {
    private CANSparkMax ElbowMotor = new CANSparkMax(Constants.Elbow.CANIDs.ELBOW_MOTOR, MotorType.kBrushless);
    private ArmFeedforward FeedForward = new ArmFeedforward(Constants.Elbow.FeedForward.kS,
    Constants.Elbow.FeedForward.kG, Constants.Elbow.FeedForward.kV, Constants.Elbow.FeedForward.kA);
    
    public Elbow(PIDController controller, double initialPosition) {
        super(controller, initialPosition);
        
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        // TODO Auto-generated method stub
        
    }

    @Override
    protected double getMeasurement() {
        // TODO Auto-generated method stub
        return 0;
    }

}