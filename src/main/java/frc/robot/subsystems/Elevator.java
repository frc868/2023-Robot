package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;


/**
 * elevator
 * 
 * @author hr jt
 */

public class Elevator extends SubsystemBase {

    private Constants constants = new Constants();

    CANSparkMax lSpool = new CANSparkMax(0, MotorType.kBrushless);
    CANSparkMax rSpool = new CANSparkMax(1, MotorType.kBrushless);
    PIDController pid = new PIDController(Constants.kP, Constants.kI, Constants.kD);



    public void actuateElevator() {

        lSpool.set();
        rSpool.set();

    }

    
    
}
