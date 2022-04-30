package frc.robot.commands.auton.paths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * Auton that does nothing.
 */
public class DoNothing extends SequentialCommandGroup {
    public DoNothing() {
        addCommands(new WaitCommand(1.0));
    }
}
