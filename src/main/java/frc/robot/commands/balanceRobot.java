package frc.robot.commands;

import frc.robot.commands.RobotStates;

import com.ctre.phoenix.sensors.Pigeon2;

import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.techhounds.houndutil.houndauto.AutoManager;
import com.techhounds.houndutil.houndauto.AutoTrajectoryCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldConstants;
import frc.robot.GamePieceLocation;
import frc.robot.GamePieceLocation.GamePiece;
import frc.robot.GridInterface;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Elbow.ElbowPosition;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorPosition;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Drivetrain.DriveMode;

/**
 *  Balances the robot by correcting pitch and roll
 *  
*/
public class balanceRobot extends CommandBase{
    private Drivetrain drivetrain;
    private Pigeon2 pigeon;
    public balanceRobot(Drivetrain drivetrain, Pigeon2 pigeon){
        this.drivetrain=drivetrain;
        this.pigeon=pigeon;
        addRequirements(drivetrain);
    }
    @Override
    public void execute(){
        double xSpeed=-1*Math.sin(pigeon.getPitch())*0.1; // some constant for gravity and friction and whatever
        double ySpeed=-1*Math.sin(pigeon.getRoll())*0.1; // ^
        drivetrain.drive(xSpeed,ySpeed,0,DriveMode.ROBOT_RELATIVE);
    }
    
    
}
