package frc.robot;

import com.techhounds.houndutil.houndlog.loggers.TunableNumber;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

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
                public static final int DRIVE_MOTOR = 1; // untested
                public static final int TURN_MOTOR = 2; // untested
                public static final int TURN_ENCODER = 0; // untested
            }

            public static final class FrontRight {
                public static final int DRIVE_MOTOR = 3; // untested
                public static final int TURN_MOTOR = 4; // untested
                public static final int TURN_ENCODER = 1; // untested
            }

            public static final class BackLeft {
                public static final int DRIVE_MOTOR = 5; // untested
                public static final int TURN_MOTOR = 6; // untested
                public static final int TURN_ENCODER = 2; // untested
            }

            public static final class BackRight {
                public static final int DRIVE_MOTOR = 7; // untested
                public static final int TURN_MOTOR = 8; // untested
                public static final int TURN_ENCODER = 3; // untested
            }
        }
        public static final class PID {
            public static final class DriveMotors {
                public static final TunableNumber kP = new TunableNumber("Drivetrain", "driveKP", 3.3657); // untested
                public static final TunableNumber kI = new TunableNumber("Drivetrain", "driveKI", 0.0); // untested
                public static final TunableNumber kD = new TunableNumber("Drivetrain", "driveKD", 0.0); // untested
                public static final double kS = 0.12817; // untested
                public static final double kV = 2.7653; // untested
                public static final double kA = 0.29499; // untested
            }

            public static final class TurnMotors {
                public static final TunableNumber kP = new TunableNumber("Drivetrain", "turnKP", 0.4); // untested
                public static final TunableNumber kI = new TunableNumber("Drivetrain", "turnKI", 0.0); // untested
                public static final TunableNumber kD = new TunableNumber("Drivetrain", "turnKD", 0.01); // untested
            }

            public static final class Trajectories {
                public static final double xkP = 1.5; // untested
                public static final double ykP = 1.5; // untested
                public static final double thetakP = 1.5; // untested
            }

            public static final class TurnToAngle {
                public static final double kP = 0.9; // untested
                public static final double kI = 0; // untested
                public static final double kD = 0; // untested
            }

        }

        public static final class Offsets {
            public static final double FRONT_LEFT = -4.654; // untested
            public static final double FRONT_RIGHT = -3.465; // untested
            public static final double BACK_LEFT = -4.433; // untested
            public static final double BACK_RIGHT = -4.580; // untested
        }

        public static final class Geometry {
            /** Distance between centers of right and left wheels on robot. */
            public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(18.9); // untested
            /** Distance between front and back wheels on robot. */
            public static final double WHEEL_BASE_METERS = Units.inchesToMeters(18.9); // untested
            public static final double GEARING = 1.0 / 6.75; // untested
            public static final double WHEEL_RADIUS_METERS = 0.048; // untested
            public static final double WHEEL_CIRCUMFERENCE = 2.0 * Math.PI * WHEEL_RADIUS_METERS;
            public static final double ENCODER_DISTANCE_TO_METERS = WHEEL_CIRCUMFERENCE * GEARING;

            public static final double MAX_PHYSICAL_VELOCITY_METERS_PER_SECOND = 4.42; // untested
            public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = Math.PI / 4; // untested
            public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 2 * Math.PI; // untested

            public static final class Turning {
                public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 5 * Math.PI;
                public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 5 * Math.PI;
            }

            public static final Translation2d[] SWERVE_MODULE_LOCATIONS = new Translation2d[] {
                    new Translation2d(WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2),
                    new Translation2d(WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2),
                    new Translation2d(-WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2),
                    new Translation2d(-WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2) };

            public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
                    SWERVE_MODULE_LOCATIONS[0],
                    SWERVE_MODULE_LOCATIONS[1],
                    SWERVE_MODULE_LOCATIONS[2],
                    SWERVE_MODULE_LOCATIONS[3]);
        }
    }
    public static final class Elbow {
        public final class CANIDs {
            public static final int ELBOW_MOTOR = 0; //untested
            public static final int ENCODER_CHANNEL = 0; //untested
            public static final int B_HALL_SENSOR_CHANNEL = 0; //untested
            public static final int T_HALL_SENSOR_CHANNEL = 0; //untested
        }
        public final class PID {
            public static final double kP = 0.0; //untested
            public static final double kI = 0.0; //untested
            public static final double kD = 0.0; //untested
        }
        public final class FeedForward {
            public static final double kS = 0.0; //untested
            public static final double kG = 0.0; //untested
            public static final double kV = 0.0; //untested
            public static final double kA = 0.0; //untested

        }
        public final class ArmState {
            public static final double INITIAL_POSITION = 0.0; //untested
        }
    }

    public static final class OI {
        public static final int DRIVER_PORT = 0;
        public static final int OPERATOR_PORT = 1;
    }

    public static final class Teleop {
        public static double PERCENT_LIMIT = 0.60; // untested
        public static final double SLOW_MODE_PERCENT_LIMIT = 0.20; // untested
        public static final boolean IS_JOYSTICK_INPUT_RATE_LIMITED = true;

        /**
         * A value inputted into the rate limiter (the joystick input) can move from 0
         * to 1 in 1/RATE_LIMIT seconds.
         * 
         * A rate limit of 3, for example, means that 0->1 in 1/3 sec.
         * Larger numbers mean less of a rate limit.
         */
        public static final double JOYSTICK_INPUT_RATE_LIMIT = 15.0; // untested

    }

    public static final class Vision {
        public static final String[] CAMERA_NAMES = new String[] { "OV9281-01", "OV9281-02", "OV9281-03", "OV9281-04" };
        public static final Transform3d[] ROBOT_TO_CAMS = new Transform3d[] {
                new Transform3d(
                        new Translation3d(0.24, 0.0, 0.0), // untested
                        new Rotation3d(0, 0, 0)), // untested
                new Transform3d(
                        new Translation3d(0.24, 0.0, 0.0), // untested
                        new Rotation3d(0, 0, 0)), // untested
                new Transform3d(
                        new Translation3d(0.24, 0.0, 0.0), // untested
                        new Rotation3d(0, 0, 0)), // untested
                new Transform3d(
                        new Translation3d(0.24, 0.0, 0.0), // untested
                        new Rotation3d(0, 0, 0)), // untested
        };
    }

    public static final class Auton {
        public static final double MAX_VELOCITY = 1; // untested
        public static final double MAX_ACCELERATION = 1; // untested
        public static final double MAX_ANGULAR_VELOCITY = 6 * Math.PI; // untested
        public static final double MAX_ANGULAR_ACCELERATION = 8 * Math.PI; // untested

    }
}
