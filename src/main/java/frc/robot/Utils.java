package frc.robot;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public class Utils {
    public static CommandBase flashButtonCommand(BooleanConsumer buttonLed) {
        return Commands.sequence(
                Commands.runOnce(() -> buttonLed.accept(true)),
                Commands.waitSeconds(0.25),
                Commands.runOnce(() -> buttonLed.accept(false)),
                Commands.waitSeconds(0.25)).repeatedly()
                .finallyDo((d) -> buttonLed.accept(false)).withName("Flash Button");
    }
}
