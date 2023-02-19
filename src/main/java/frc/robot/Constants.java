package frc.robot;

import com.techhounds.houndutil.houndlog.loggers.TunableNumber;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

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

    public static final double ROBOT_SIDE_LENGTH = Units.inchesToMeters(34.75);

    public static final boolean IS_USING_CAMERAS = true;
    public static boolean IS_NT_COMMANDS_ENABLED = RobotBase.isSimulation();
    public static boolean IS_VIRTUAL_BUTTON_PANEL_ENABLED = RobotBase.isSimulation();

    public static final class CAN {
        public static final int FRONT_LEFT_DRIVE_MOTOR = 1;
        public static final int FRONT_LEFT_TURN_MOTOR = 2;
        public static final int FRONT_RIGHT_DRIVE_MOTOR = 4;
        public static final int FRONT_RIGHT_TURN_MOTOR = 3;
        public static final int BACK_LEFT_DRIVE_MOTOR = 6;
        public static final int BACK_LEFT_TURN_MOTOR = 5;
        public static final int BACK_RIGHT_DRIVE_MOTOR = 7;
        public static final int BACK_RIGHT_TURN_MOTOR = 8;

        public static final int ELEVATOR_LEFT_MOTOR = 9;
        public static final int ELEVATOR_RIGHT_MOTOR = 10;
        public static final int ELBOW_MOTOR = 11;
        public static final int PASSOVER_LEFT_MOTOR = 12;
        public static final int PASSOVER_RIGHT_MOTOR = 13;

        public static final int FRONT_LEFT_TURN_ENCODER = 0;
        public static final int FRONT_RIGHT_TURN_ENCODER = 1;
        public static final int BACK_LEFT_TURN_ENCODER = 2;
        public static final int BACK_RIGHT_TURN_ENCODER = 3;
    }

    public static final class Pneumatics {
        public static final int[] PINCERS = { 3, 2 };
        public static final int[] WRIST = { 4, 5 };
        public static final int[] INTAKE = { 1, 0 };
        public static final int[] PASSOVER = { 9, 8 };
    }

    public static final class DIO {
        public static final int ELEVATOR_TOP_LIMIT = 0;
        public static final int ELEVATOR_BOTTOM_LIMIT = 1;
        public static final int ELBOW_TOP_LIMIT = 2;
        public static final int ELBOW_BOTTOM_LIMIT = 3;
        public static final int POLE_SWITCH = 4;
        public static final int GAME_PIECE_SENSOR = 5;
    }

    public static final class Gains {
        public static final class DriveMotors {
            public static final TunableNumber kP = new TunableNumber("Drivetrain", "driveKP", 0.12575);
            public static final TunableNumber kI = new TunableNumber("Drivetrain", "driveKI", 0.0);
            public static final TunableNumber kD = new TunableNumber("Drivetrain", "driveKD", 0.0);
            public static final double kS = 0.2089;
            public static final double kV = 2.7069;
            public static final double kA = 0.41713;
        }

        public static final class TurnMotors {
            public static final TunableNumber kP = new TunableNumber("Drivetrain", "turnKP", 0.4); // untested
            public static final TunableNumber kI = new TunableNumber("Drivetrain", "turnKI", 0.0); // untested
            public static final TunableNumber kD = new TunableNumber("Drivetrain", "turnKD", 0.01); // untested
        }

        public static final class Trajectories {
            public static final double xkP = 5; // untested
            public static final double ykP = 5; // untested
            public static final double thetakP = 1.5; // untested
        }

        public static final class TurnToAngle {
            public static final double kP = 0.9; // untested
            public static final double kI = 0; // untested
            public static final double kD = 0; // untested
        }

        public static final class Elevator {
            public static final TunableNumber kP = new TunableNumber("Elevator", "kP", 80.06);
            public static final TunableNumber kI = new TunableNumber("Elevator", "kI", 0);
            public static final TunableNumber kD = new TunableNumber("Elevator", "kD", 5.0933);
            public static final TunableNumber TOLERANCE = new TunableNumber("Elevator", "Tolerance", 0.05);
            public static final double kS = 0.12;
            public static final double kG = 0.113;
            public static final double kV = 8.6264;
            public static final double kA = 0.299153;
        }

        public static final class Elbow {
            public static final TunableNumber kP = new TunableNumber("Elbow", "kP", 3.5);
            public static final TunableNumber kI = new TunableNumber("Elbow", "kI", 0);
            public static final TunableNumber kD = new TunableNumber("Elbow", "kD", 0.89);
            public static final TunableNumber TOLERANCE = new TunableNumber("Elbow", "Tolerance", 0.07);
            public static final double kS = 0.31;
            public static final double kG = 0.54038;
            public static final double kV = 0.85;
            public static final double kA = 0.041;
        }

    }

    public static final class Geometries {
        public static final class Drivetrain {

            public static final class Offsets {
                public static final double FRONT_LEFT = -4.654;
                public static final double FRONT_RIGHT = -3.465;
                public static final double BACK_LEFT = -4.433;
                public static final double BACK_RIGHT = -4.580;
            }

            /** Distance between centers of right and left wheels on robot. */
            public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(22.75);
            /** Distance between front and back wheels on robot. */
            public static final double WHEEL_BASE_METERS = Units.inchesToMeters(22.75);
            public static final double GEARING = 1.0 / 6.75;
            public static final double WHEEL_RADIUS_METERS = 0.048;
            public static final double WHEEL_CIRCUMFERENCE = 2.0 * Math.PI * WHEEL_RADIUS_METERS;
            public static final double ENCODER_DISTANCE_TO_METERS = WHEEL_CIRCUMFERENCE * GEARING;

            public static final double MAX_PHYSICAL_VELOCITY_METERS_PER_SECOND = 4.42;
            public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = Math.PI / 4;
            public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 2 * Math.PI;

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

        public static final class Elevator {
            public static final TunableNumber MAX_VELOCITY_METERS_PER_SECOND = new TunableNumber("Elevator",
                    "Max Velocity", 1.2); // untested
            public static final TunableNumber MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = new TunableNumber("Elevator",
                    "Max Acceleration", 2); // untested

            public static final double GEARING = 1.0 / 12.0; // untested
            public static final double DRUM_RADIUS_METERS = Units.inchesToMeters(1.2); // untested // 0.0305
            public static final double WHEEL_CIRCUMFERENCE = 2.0 * Math.PI * DRUM_RADIUS_METERS;
            public static final double ENCODER_DISTANCE_TO_METERS = WHEEL_CIRCUMFERENCE * GEARING;

        }

        public static final class Elbow {
            public static final TunableNumber MAX_VELOCITY_METERS_PER_SECOND = new TunableNumber("Elbow",
                    "Max Velocity",
                    1.5 * Math.PI); // untested
            public static final TunableNumber MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = new TunableNumber("Elbow",
                    "Max Acceleration", 1.5 * Math.PI); // untested
        }
    }

    public static final class LEDs {
        public static final int PORT = 0;
        public static final int LENGTH = 50;
    }

    public static final class Teleop {
        public static double SLOW_PERCENT_LIMIT = 0.25; // untested
        public static double FAST_PERCENT_LIMIT = 0.65; // untested
        public static double ULTRA_PERCENT_LIMIT = 1.0; // untested
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
        public static final String[] CAMERA_NAMES = new String[] {
                "OV9281-03",
                "OV9281-04",
                "OV9281-01",
                // "OV9281-02"
        };
        public static final Transform3d[] ROBOT_TO_CAMS = new Transform3d[] {
                new Transform3d(
                        new Translation3d(Units.inchesToMeters(10.125), -Units.inchesToMeters(7.75),
                                Units.inchesToMeters(21.75)),
                        new Rotation3d(0, 0, 0)),
                new Transform3d(
                        new Translation3d(-Units.inchesToMeters(11), -Units.inchesToMeters(5.5),
                                Units.inchesToMeters(26.8)),
                        new Rotation3d(0, 0, -Math.PI / 2.0)),
                new Transform3d(
                        new Translation3d(-Units.inchesToMeters(12.625), -Units.inchesToMeters(6.25),
                                Units.inchesToMeters(26.3)),
                        new Rotation3d(0, 0, Math.PI)),
                // new Transform3d(
                // new Translation3d(-Units.inchesToMeters(11), -Units.inchesToMeters(8),
                // Units.inchesToMeters(26.8)),
                // new Rotation3d(0, 0, Math.PI / 2.0)),
        };
    }
}
