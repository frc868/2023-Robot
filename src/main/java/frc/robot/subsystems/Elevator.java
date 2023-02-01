package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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

import com.techhounds.houndutil.houndauto.AutoManager;
import com.techhounds.houndutil.houndlib.AprilTagPhotonCamera;
import com.techhounds.houndutil.houndlog.LogGroup;
import com.techhounds.houndutil.houndlog.LogProfileBuilder;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.loggers.DeviceLogger;
import com.techhounds.houndutil.houndlog.loggers.Logger;
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

    private SimpleMotorFeedforward elevatorFeedforward = new SimpleMotorFeedforward(Constants.Elevator.PID.kS, Constants.Elevator.PID.kV);

    private ProfiledPIDController elevatorController = new ProfiledPIDController(Constants.Elevator.PID.kP.get(),
            Constants.Elevator.PID.kI.get(),
            Constants.Elevator.PID.kD.get(),
            new TrapezoidProfile.Constraints(Constants.Elevator.MAX_VELOCITY,
                    Constants.Elevator.MAX_ACCELERATION));

    public Elevator(){

        LoggingManager.getInstance().addGroup("Elevator", new LogGroup(
                new Logger[] {
                        new DeviceLogger<CANSparkMax>(primary, "Primary Elevator Motor",
                            LogProfileBuilder.buildCANSparkMaxLogItems(primary)),
                        new DeviceLogger<CANSparkMax>(secondary, "Secondary Elevator Motor",
                                LogProfileBuilder.buildCANSparkMaxLogItems(secondary))
                }));

    }

    public void setElevatorPosition(double position){
        elevatorController.setGoal(position);
    }

    public void extendElevator(){
        setElevatorPosition(Constants.Elevator.MAX_LENGTH);
    }

    public void retractElevator(){
        setElevatorPosition(Constants.Elevator.MIN_LENGTH);
    }

    public void setSpeed(double speed){
        if(!topLimit.get()){
            primary.set(0);
            secondary.set(0);
        }
        if(!bottomLimit.get()){
            primary.set(0);
            secondary.set(0);
        }
        primary.set(speed);
        secondary.set(speed);
    }
    

    


}
