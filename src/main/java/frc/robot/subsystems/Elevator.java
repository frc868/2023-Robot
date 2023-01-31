package frc.robot.subsystems;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;


import frc.robot.Constants;

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
