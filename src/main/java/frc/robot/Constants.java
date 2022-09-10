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

    public static final ControllerType CONTROLLER_TYPE = ControllerType.FlightStick;

    public static final class Drivetrain {
        public static final class CANIDs {
            public static final class FrontLeft {
                public static final int DRIVE_MOTOR = 1;
                public static final int TURN_MOTOR = 2;
                public static final int TURN_ENCODER = 0;
            }

            public static final class FrontRight {
                public static final int DRIVE_MOTOR = 3;
                public static final int TURN_MOTOR = 4;
                public static final int TURN_ENCODER = 1;
            }

            public static final class BackLeft {
                public static final int DRIVE_MOTOR = 5;
                public static final int TURN_MOTOR = 6;
                public static final int TURN_ENCODER = 2;
            }

            public static final class BackRight {
                public static final int DRIVE_MOTOR = 7;
                public static final int TURN_MOTOR = 8;
                public static final int TURN_ENCODER = 3;
            }
        }

        public static final class PID {
            public static final class Drive {
                public static final double kP = 1.0; // recommended by SDS
                public static final double kI = 0.0; // recommended by SDS
                public static final double kD = 0.1; // recommended by SDS
                public static final double kS = 0.0;
                public static final double kV = 0.0;
            }

            public static final class Turn {
                public static final double kP = 1.0; // recommended by SDS
                public static final double kI = 0.0; // recommended by SDS
                public static final double kD = 0.1; // recommended by SDS
                public static final double kS = 0.0;
                public static final double kV = 0.0;
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

        public static final class Geometry {
            /** Distance between centers of right and left wheels on robot. */
            public static final double TRACK_WIDTH_METERS = 0.625475;
            /** Distance between front and back wheels on robot. */
            public static final double WHEEL_BASE_METERS = 0.625475;
            public static final double WHEEL_RADIUS_METERS = 0.0;
            public static final double MAX_PHYSICAL_VELOCITY_METERS_PER_SECOND = 4.42;
            public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = Math.PI / 4;
            public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 2 * Math.PI;

            public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
                    new Translation2d(WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2),
                    new Translation2d(WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2),
                    new Translation2d(-WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2),
                    new Translation2d(-WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2));
        }
    }

    public static final class OI {
        public static final int DRIVER_PORT = 0;
        public static final int OPERATOR_PORT = 1;
    }

    public static final class Teleop {
        public static final double PERCENT_LIMIT = 0.20;
    }

    public static final class Auton {
        public static final double MAX_VELOCITY = 1;
        public static final double MAX_ACCELERATION = 1;
        public static final double MAX_ANGULAR_VELOCITY = Math.PI / 4;
        public static final double MAX_ANGULAR_ACCELERATION = Math.PI;

    }
}
