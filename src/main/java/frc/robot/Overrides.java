package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public class Overrides {
    private static boolean OPERATOR_OVERRIDE = false;

    public static CommandBase operatorOverrideCommand() {
        return Commands.startEnd(() -> OPERATOR_OVERRIDE = true, () -> OPERATOR_OVERRIDE = false)
                .withName("Operator Override");
    }

    public static boolean isOperatorOverridden() {
        return OPERATOR_OVERRIDE;
    }
}
