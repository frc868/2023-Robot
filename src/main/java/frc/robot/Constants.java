package frc.robot;

import com.techhounds.houndutil.houndlib.swerve.NEOCoaxialSwerveModule;
import com.techhounds.houndutil.houndlog.logitems.TunableNumber;

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

    public static final boolean IS_USING_CAMERAS = false;
    public static boolean IS_NT_COMMANDS_ENABLED = true;
    public static boolean IS_VIRTUAL_BUTTON_PANEL_ENABLED = RobotBase.isSimulation();

    public static final TunableNumber DRIVE_RATE_LIMIT = new TunableNumber("Drivetrain", "Drive Rate Limit", 0.2);

    public static final class CAN {
        public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 1;
        public static final int FRONT_LEFT_TURN_MOTOR_ID = 2;
        public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 3;
        public static final int FRONT_RIGHT_TURN_MOTOR_ID = 4;
        public static final int BACK_LEFT_DRIVE_MOTOR_ID = 5;
        public static final int BACK_LEFT_TURN_MOTOR_ID = 6;
        public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 7;
        public static final int BACK_RIGHT_TURN_MOTOR_ID = 8;

        public static final int ELEVATOR_LEFT_MOTOR_ID = 9;
        public static final int ELEVATOR_RIGHT_MOTOR_ID = 10;
        public static final int ELBOW_MOTOR_ID = 11;
        public static final int PASSOVER_LEFT_MOTOR_ID = 12;
        public static final int PASSOVER_RIGHT_MOTOR_ID = 13;

        public static final int FRONT_LEFT_TURN_ENCODER_ID = 0;
        public static final int FRONT_RIGHT_TURN_ENCODER_ID = 1;
        public static final int BACK_LEFT_TURN_ENCODER_ID = 2;
        public static final int BACK_RIGHT_TURN_ENCODER_ID = 3;
    }

    public static final class Pneumatics {
        // forward, reverse
        public static final int[] PASSOVER_PORTS = { 3, 12 };
        public static final int[] WRIST_PORTS = { 2, 13 };
        public static final int[] PINCERS_PORTS = { 1, 14 };
        public static final int[] CUBAPULT_PORTS = { 0, 15 };
    }

    public static final class DIO {
        public static final int ELEVATOR_BOTTOM_LIMIT = 0;
        public static final int POLE_SWITCH = 1;

        public static final int ELEVATOR_TOP_LIMIT = 2; // unused
        public static final int ELBOW_TOP_LIMIT = 3; // unused
        public static final int ELBOW_BOTTOM_LIMIT = 4; // unused
        public static final int GAME_PIECE_SENSOR = 5; // unused
    }

    public static final class Gains {
        public static final class DriveMotors {
            public static final double kP = 0.12575;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double kS = 0.2089;
            public static final double kV = 2.7069;
            public static final double kA = 0.41713;
        }

        public static final class TurnMotors {
            public static final double kP = 0.4; // untested
            public static final double kI = 0.0; // untested
            public static final double kD = 0.01; // untested
        }

        public static final class Trajectories {
            public static final double xkP = 5; // untested
            public static final double ykP = 5; // untested
            public static final double thetakP = 1; // untested
        }

        public static final class TurnToAngle {
            public static final double kP = 1.1; // untested
            public static final double kI = 0; // untested
            public static final double kD = 0; // untested
        }

        public static final class Elevator {
            public static final double kP = 60;
            public static final double kI = 0;
            public static final double kD = 0;
            public static final double TOLERANCE = 0.015;
            public static final double kS = 0.25696;
            public static final double kG = 0.20167;
            public static final double kV = 2.3833;
            public static final double kA = 0.29009;
        }

        public static final class Elbow {
            public static final double kP = 8;
            public static final double kI = 0;
            public static final double kD = 0.5;
            public static final double TOLERANCE = 0.05;
            public static final double kS = 0.33055;
            public static final double kG = 0.54418;
            public static final double kV = 0.74458;
            public static final double kA = 0.044339;
        }

    }

    public static final class Geometries {
        public static final class Drivetrain {
            public static final class Offsets {
                public static final double FRONT_LEFT = -4.63107;
                public static final double FRONT_RIGHT = -3.5587;
                public static final double BACK_LEFT = -4.4377;
                public static final double BACK_RIGHT = -6.2019;
            }

            /** Distance between centers of right and left wheels on robot. */
            public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(22.75);
            /** Distance between front and back wheels on robot. */
            public static final double WHEEL_BASE_METERS = Units.inchesToMeters(22.75);
            public static final double GEARING = 1.0 / 6.75;
            public static final double TURN_GEARING = 1.0 / (150.0 / 7.0);
            public static final double WHEEL_RADIUS_METERS = 0.048;
            public static final double WHEEL_CIRCUMFERENCE = 2.0 * Math.PI * WHEEL_RADIUS_METERS;
            public static final double ENCODER_DISTANCE_TO_METERS = WHEEL_CIRCUMFERENCE * GEARING;
            public static final double TURN_ENCODER_DISTANCE_TO_METERS = 2 * Math.PI * TURN_GEARING;

            public static final double MAX_DRIVING_VELOCITY_METERS_PER_SECOND = 4.42;
            public static final double MAX_DRIVING_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;

            // for drivetrain
            public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = Math.PI / 4;
            public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 2 * Math.PI;
            // for module wheel
            public static final double MAX_MODULE_TURNING_VELOCITY_RADIANS_PER_SECOND = 5 * Math.PI;
            public static final double MAX_MODULE_TURNING_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 5 * Math.PI;

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

            public static final NEOCoaxialSwerveModule.SwerveConstants SWERVE_CONSTANTS = new NEOCoaxialSwerveModule.SwerveConstants(
                    Gains.DriveMotors.kP,
                    Gains.DriveMotors.kI,
                    Gains.DriveMotors.kD,
                    Gains.DriveMotors.kS,
                    Gains.DriveMotors.kV,
                    Gains.DriveMotors.kA,
                    Gains.TurnMotors.kP,
                    Gains.TurnMotors.kI,
                    Gains.TurnMotors.kD,
                    GEARING,
                    TURN_GEARING,
                    WHEEL_RADIUS_METERS,
                    MAX_DRIVING_VELOCITY_METERS_PER_SECOND,
                    MAX_DRIVING_ACCELERATION_METERS_PER_SECOND_SQUARED,
                    MAX_MODULE_TURNING_VELOCITY_RADIANS_PER_SECOND,
                    MAX_MODULE_TURNING_ACCELERATION_RADIANS_PER_SECOND_SQUARED,
                    ENCODER_DISTANCE_TO_METERS,
                    TURN_ENCODER_DISTANCE_TO_METERS);
        }

        public static final class Elevator {
            public static final double MAX_VELOCITY_METERS_PER_SECOND = 1; // untested
            public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 1; // untested
            public static final double MAX_STOWING_VELOCITY_METERS_PER_SECOND = 2.6; // untested
            public static final double MAX_STOWING_ACCELERATION_METERS_PER_SECOND_SQUARED = 4.0; // untested

            public static final double GEARING = 1.0 / 4.0; // untested
            public static final double DRUM_RADIUS_METERS = Units.inchesToMeters(1.2); // untested // 0.0305
            public static final double WHEEL_CIRCUMFERENCE = 2.0 * Math.PI * DRUM_RADIUS_METERS;
            public static final double ENCODER_DISTANCE_TO_METERS = WHEEL_CIRCUMFERENCE * GEARING;

        }

        public static final class Elbow {
            public static final double MAX_VELOCITY_METERS_PER_SECOND = 18 * Math.PI; // untested
            public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 10 * Math.PI;// untested
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
        // front-left, front-right, back-left, back-right
        public static final Transform3d[] ROBOT_TO_CAMS = new Transform3d[] {
                new Transform3d(
                        new Translation3d(Units.inchesToMeters(-10.1376), Units.inchesToMeters(-5.3876),
                                Units.inchesToMeters(28.826)),
                        new Rotation3d(0, 0, Math.PI / 4.0)),
                new Transform3d(
                        new Translation3d(Units.inchesToMeters(-10.1376), Units.inchesToMeters(-7.8624),
                                Units.inchesToMeters(28.826)),
                        new Rotation3d(0, 0, -Math.PI / 4.0)),

                new Transform3d(
                        new Translation3d(Units.inchesToMeters(-12.6124), Units.inchesToMeters(-5.3876),
                                Units.inchesToMeters(28.826)),
                        new Rotation3d(0, 0, 3.0 * Math.PI / 4.0)),
                new Transform3d(
                        new Translation3d(Units.inchesToMeters(-12.6124), Units.inchesToMeters(-7.8624),
                                Units.inchesToMeters(28.826)),
                        new Rotation3d(0, 0, -3.0 * Math.PI / 4.0)),
        };
    }
}
// 2.298