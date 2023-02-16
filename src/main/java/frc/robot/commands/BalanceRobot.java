package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import com.ctre.phoenix.sensors.Pigeon2;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.DriveMode;

/**
 * Simple command to keep the robot balanced on the charge station
 * 
 * @author jt
 */
public class BalanceRobot extends CommandBase {
    private Drivetrain drivetrain;
    private Pigeon2 pigeon;

    public BalanceRobot(Drivetrain drivetrain, Pigeon2 pigeon) {
        this.drivetrain = drivetrain;
        this.pigeon = pigeon;
        addRequirements(drivetrain);
    }

    /**
     * attempts to move the robot against the force of gravity based on its pitch
     * (and roll just
     * in case)
     */
    @Override
    public void execute() {
        double xSpeed = -1 * Math.sin(pigeon.getPitch()) * 0.1; // some constant for gravity and
                                                                // friction and whatever (untested)
        double ySpeed = -1 * Math.sin(pigeon.getRoll()) * 0.1; // ^
        drivetrain.drive(xSpeed, ySpeed, 0, DriveMode.ROBOT_RELATIVE);
    }

}
