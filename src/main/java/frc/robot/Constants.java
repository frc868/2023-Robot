package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The container for robot-wide numerical or boolean constants. This should not
 * be used for any other purpose.
 *
 * @author dr
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

        public static final class Geometry {
            /** In m */
            public static final double WHEEL_RADIUS = 0.0;
            /** In m */
            public static final double CHASSIS_WIDTH = 0.5; // for square base only
            /** In m/s */
            public static final double MAX_VELOCITY = 4.42;
            /** In m/s/s */
            public static final double MAX_ACCELERATION = 1.0;
            /** In rad/s */
            public static final double MAX_ANGULAR_VELOCITY = Math.PI / 4;
            /** In rad/s/s */
            public static final double MAX_ANGULAR_ACCELERATION = 2 * Math.PI;

            public static final Translation2d FRONT_LEFT_LOCATION = new Translation2d(
                    CHASSIS_WIDTH / 2,
                    CHASSIS_WIDTH / 2);
            public static final Translation2d FRONT_RIGHT_LOCATION = new Translation2d(
                    CHASSIS_WIDTH / 2,
                    -CHASSIS_WIDTH / 2);
            public static final Translation2d BACK_LEFT_LOCATION = new Translation2d(
                    -CHASSIS_WIDTH / 2,
                    CHASSIS_WIDTH / 2);
            public static final Translation2d BACK_RIGHT_LOCATION = new Translation2d(
                    -CHASSIS_WIDTH / 2,
                    -CHASSIS_WIDTH / 2);

            public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
                    FRONT_LEFT_LOCATION,
                    FRONT_RIGHT_LOCATION,
                    BACK_LEFT_LOCATION,
                    BACK_RIGHT_LOCATION);
        }

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
        public static final double MAX_ANGULAR_VELOCITY = Math.PI / 4;
        public static final double MAX_ANGULAR_ACCELERATION = Math.PI;

    }
}
