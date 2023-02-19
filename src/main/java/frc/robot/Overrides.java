package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public enum Overrides {
    FRONT_LEFT_DRIVE_MOTOR_DISABLE(false),
    FRONT_LEFT_TURN_MOTOR_DISABLE(false),
    FRONT_RIGHT_DRIVE_MOTOR_DISABLE(false),
    FRONT_RIGHT_TURN_MOTOR_DISABLE(false),
    BACK_LEFT_DRIVE_MOTOR_DISABLE(false),
    BACK_LEFT_TURN_MOTOR_DISABLE(false),
    BACK_RIGHT_DRIVE_MOTOR_DISABLE(false),
    BACK_RIGHT_TURN_MOTOR_DISABLE(false),

    ELEVATOR_LEFT_MOTOR_DISABLE(false),
    ELEVATOR_RIGHT_MOTOR_DISABLE(false),
    ELBOW_MOTOR_DISABLE(false),
    PASSOVER_LEFT_MOTOR_DISABLE(false),
    PASSOVER_RIGHT_MOTOR_DISABLE(false),

    MANUAL_MECH_CONTROL_MODE(false),
    SAFETIES_DISABLE(true),
    SPEED_LIMITS_DISABLE(true),
    MECH_LIMITS_DISABLE(true),
    DRIVER_EMERGENCY_MODE(false);

    private boolean isEnabled;

    private Overrides(boolean isEnabled) {
        this.isEnabled = isEnabled;
    }

    public boolean getStatus() {
        return this.isEnabled;
    }

    public BooleanSupplier getStatusSupplier() {
        return () -> this.isEnabled;
    }

    public CommandBase enableC() {
        return Commands.runOnce(() -> this.isEnabled = true);
    }

    public CommandBase disableC() {
        return Commands.runOnce(() -> this.isEnabled = true);
    }
}
