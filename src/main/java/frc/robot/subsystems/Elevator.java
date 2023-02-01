package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
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

public class Elevator extends SubsystemBase{
    private CANSparkMax primary = new CANSparkMax(Constants.Elevator.CANIDs.PRIMARY_MOTOR, MotorType.kBrushless);
    private CANSparkMax secondary = new CANSparkMax(Constants.Elevator.CANIDs.SECONDARY_MOTOR, MotorType.kBrushless);

    private DigitalInput topLimit = new DigitalInput(Constants.Elevator.CANIDs.ELEVATOR_TOP);
    private DigitalInput bottomLimit = new DigitalInput(Constants.Elevator.CANIDs.ELEVATOR_BOTTOM);

    private ProfiledPIDController elevatorController = new ProfiledPIDController(Constants.Elevator.PID.kP.get(),
            Constants.Elevator.PID.kI.get(),
            Constants.Elevator.PID.kD.get(),
            new TrapezoidProfile.Constraints(Constants.Elevator.MAX_VELOCITY,
                    Constants.Elevator.MAX_ACCELERATION));

    public Elevator(){

    }

// Two different positions for the elevator
  State eleUp = new TrapezoidProfile.State(Constants.Elevator.ELEVATOR_HEIGHT, 0);
  State eleDown = new TrapezoidProfile.State(0, 0);
  TrapezoidProfile elevatorProfile = new TrapezoidProfile(null, eleDown, eleUp);


}
