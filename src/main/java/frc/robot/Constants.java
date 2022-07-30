// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    enum ControllerType {
        XboxController,
        FlightStick
    }

    public static final boolean DEBUG_MODE = false;

    public static final ControllerType CONTROLLER_TYPE = ControllerType.XboxController;

    public static final class Drivetrain {
        public static final class CANIDs {
            public static final class FrontLeft {
                public static final int DRIVE_MOTOR = 2;
                public static final int TURN_MOTOR = 3;
                public static final int TURN_ENCODER = 10;
            }

            public static final class FrontRight {
                public static final int DRIVE_MOTOR = 4;
                public static final int TURN_MOTOR = 5;
                public static final int TURN_ENCODER = 11;
            }

            public static final class BackLeft {
                public static final int DRIVE_MOTOR = 6;
                public static final int TURN_MOTOR = 7;
                public static final int TURN_ENCODER = 12;
            }

            public static final class BackRight {
                public static final int DRIVE_MOTOR = 8;
                public static final int TURN_MOTOR = 9;
                public static final int TURN_ENCODER = 13;
            }
        }

        public static final double WHEEL_RADIUS = 0.0;
        public static final double MAX_VELOCITY = 1.0;
        public static final double MAX_ACCELERATION = 1.0;
        public static final double MAX_ANGULAR_VELOCITY = Math.PI / 4; // must be in rad/s, this is intentionally very
                                                                       // slow for now
        public static final double MAX_ANGULAR_ACCELERATION = 2 * Math.PI;
        public static final double CHASSIS_WIDTH = 0.5; // for square base only

        public static final class PIDConstants {
            public static final class Drive {
                public static final double kP = 1.0; // recommended by SDS
                public static final double kI = 0.0; // recommended by SDS
                public static final double kD = 0.1; // recommended by SDS
            }

            public static final class Turn {
                public static final double kP = 1.0; // recommended by SDS
                public static final double kI = 0.0; // recommended by SDS
                public static final double kD = 0.1; // recommended by SDS
            }

            public static final class DriveStraight {
                public static final double kP = 1.0; // recommended by SDS
                public static final double kI = 0.0; // recommended by SDS
                public static final double kD = 0.1; // recommended by SDS
            }

            public static final class TurnToAngle {
                public static final int kP = 1;
                public static final int kI = 0;
                public static final int kD = 0;
            }

            public static final class TurnToBall {
                public static final int kP = 1;
                public static final int kI = 0;
                public static final int kD = 0;
            }

            public static final class TurnToGoal {
                public static final int kP = 1;
                public static final int kI = 0;
                public static final int kD = 0;
            }

            public static final class Trajectories {
                public static final class X {
                    public static final int kP = 1;
                    public static final int kI = 0;
                    public static final int kD = 0;
                }

                public static final class Y {
                    public static final int kP = 1;
                    public static final int kI = 0;
                    public static final int kD = 0;
                }

                public static final class Theta {
                    public static final int kP = 1;
                    public static final int kI = 0;
                    public static final int kD = 0;
                }

            }
        }
    }

    public static final class OI {
        public static final int DRIVER_PORT = 0;
        public static final int OPERATOR_PORT = 1;
    }

    public static final class Hopper {
        public static final class CANIDs {
            public static final int MOTOR = 10;
        }

        public static final class Solenoids {
            public static final int GATEKEEPER_CHANNEL_1 = 1;
            public static final int GATEKEEPER_CHANNEL_2 = 6;
        }

        public static final boolean IS_INVERTED = false;
    }

    public static final class Intake {
        public static final class CANIDs {
            public static final int MOTOR = 11;
        }

        public static final class Solenoids {
            public static final int INTAKE_CHANNEL_1 = 0;
            public static final int INTAKE_CHANNEL_2 = 7;
        }

        public static final boolean IS_INVERTED = false;
    }

    public static final class Shooter {
        public static final class CANIDs {
            public static final int PRIMARY = 12;
            public static final int SECONDARY = 13;
        }

        public static final boolean IS_INVERTED = false;
        public static final int kP = 1;
        public static final int kI = 0;
        public static final int kD = 0;
        public static final int kS = 0;
        public static final int kV = 0;
    }

    public static final class Limelight {

        // Goal and angle measurements
        public static final double LL_HEIGHT = 22.0;
        public static final double HUB_HEIGHT = 101.0;
        public static final double ANGLE = 29.6375;

        // Good shot distances
        public static final double HIGH_GOAL_SHOT_DISTANCE = 6.75;
        public static final double LOW_GOAL_SHOT_DISTANCE = 0.0; // untested
    }

    public static final class Climber {
        public static final class CANIDs {
            public static final int PRIMARY = 14;
            public static final int SECONDARY = 15;
        }

        public static final boolean IS_INVERTED = false;

        public static final class Solenoids {
            public static final int CLIMBER_2ND_STAGE_CHANNEL_1 = 2;
            public static final int CLIMBER_2ND_STAGE_CHANNEL_2 = 5;
            public static final int CLIMBER_LOCK_CHANNEL_1 = 3;
            public static final int CLIMBER_LOCK_CHANNEL_2 = 4;

        }
    }

    public static final class Auton {
        public static final double MAX_VELOCITY = 1;
        public static final double MAX_ACCELERATION = 1;

    }
}
