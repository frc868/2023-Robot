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
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.techhounds.houndutil.houndauto.AutoManager;
import com.techhounds.houndutil.houndlog.LogGroup;
import com.techhounds.houndutil.houndlog.LogProfileBuilder;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.loggers.DeviceLogger;
import com.techhounds.houndutil.houndlog.loggers.Logger;
import frc.robot.Constants;

/**
 * The elevator subsystem, with two motors and motion profling.
 * 
 * @author hr jt
 */

public class Elevator extends ProfiledPIDSubsystem{
    /** The top motor. */
    private CANSparkMax primary = new CANSparkMax(Constants.Elevator.CANIDs.PRIMARY_MOTOR, MotorType.kBrushless);

    /** The bottom motor. */
    private CANSparkMax secondary = new CANSparkMax(Constants.Elevator.CANIDs.SECONDARY_MOTOR, MotorType.kBrushless);

    /** Limit switch that represents max extension.  */
    private DigitalInput topLimit = new DigitalInput(Constants.Elevator.CANIDs.ELEVATOR_TOP);

    /** Limit switch that represents minimum extension. */
    private DigitalInput bottomLimit = new DigitalInput(Constants.Elevator.CANIDs.ELEVATOR_BOTTOM);

    /** Feedforward controller. */
    private SimpleMotorFeedforward elevatorFeedforward = new SimpleMotorFeedforward(Constants.Elevator.PID.kS, Constants.Elevator.PID.kV);

    /** Initialization code. */
    public Elevator(){
        super(new ProfiledPIDController(Constants.Elevator.PID.kP.get(),
                Constants.Elevator.PID.kI.get(),
                Constants.Elevator.PID.kD.get(),
                new TrapezoidProfile.Constraints(Constants.Elevator.MAX_VELOCITY,
                    Constants.Elevator.MAX_ACCELERATION)));
            
            LoggingManager.getInstance().addGroup("Elevator", new LogGroup(
                new Logger[] {
                        new DeviceLogger<CANSparkMax>(primary, "Primary Elevator Motor",
                            LogProfileBuilder.buildCANSparkMaxLogItems(primary)),
                        new DeviceLogger<CANSparkMax>(secondary, "Secondary Elevator Motor",
                                LogProfileBuilder.buildCANSparkMaxLogItems(secondary))
                }));
    }

    @Override
    public void useOutput(double output, TrapezoidProfile.State setpoint) {
        double feedforward = elevatorFeedforward.calculate(setpoint.position, setpoint.velocity);
        primary.setVoltage(output+feedforward);
        secondary.setVoltage(output+feedforward);
    }

    @Override
    public double getMeasurement(){
        return primary.getEncoder().getPosition();
    }

    /** Sets the value of each encoder to the minimum distance. */
    public void resetEncoders(){
        primary.getEncoder().setPosition(Constants.Elevator.MIN_LENGTH);
        secondary.getEncoder().setPosition(Constants.Elevator.MIN_LENGTH);
    }

    /** Stops both motors. */
    public void stop(){
        primary.set(0);
        secondary.set(0);
    }

    /** Sets the speed of each motors, while neither limit switch has been activated. */
    public void setSpeed(double speed){
        if(topLimit.get() && bottomLimit.get()){
            primary.set(speed);
            secondary.set(speed);
        }
        else{
            stop();
        }
    }

    /** Moves the elevator to a desired position.
     * 
     * 
     * @param position the extension distance in meters.
     */
    public Command setElevatorPositionCommand(double position){
        return new RunCommand(() -> {
            if(topLimit.get() && bottomLimit.get()){
                this.setGoal(position);
            }
            else{
                stop();
            }
        });
    }

    

    


}
