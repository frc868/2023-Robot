package frc.robot;

import com.techhounds.houndutil.houndlib.AprilTagPhotonCamera.PhotonCameraConstants;
import com.techhounds.houndutil.houndlib.swerve.NEOCoaxialSwerveModule;
import com.techhounds.houndutil.houndlog.logitems.TunableDouble;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

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

    public static final class Drivetrain {
        public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 1;
        public static final int FRONT_LEFT_STEER_MOTOR_ID = 2;
        public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 3;
        public static final int FRONT_RIGHT_STEER_MOTOR_ID = 4;
        public static final int BACK_LEFT_DRIVE_MOTOR_ID = 5;
        public static final int BACK_LEFT_STEER_MOTOR_ID = 6;
        public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 7;
        public static final int BACK_RIGHT_STEER_MOTOR_ID = 8;

        public static final int FRONT_LEFT_STEER_ENCODER_ID = 0;
        public static final int FRONT_RIGHT_STEER_ENCODER_ID = 1;
        public static final int BACK_LEFT_STEER_ENCODER_ID = 2;
        public static final int BACK_RIGHT_STEER_ENCODER_ID = 3;

        public static final double FRONT_LEFT_OFFSET = -4.63107;
        public static final double FRONT_RIGHT_OFFSET = -3.5587;
        public static final double BACK_LEFT_OFFSET = -4.4377;
        public static final double BACK_RIGHT_OFFSET = -6.2019;

        /** Distance between left and right wheels. */
        public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(22.75);
        /** Distance between front and back wheels. */
        public static final double WHEEL_BASE_METERS = Units.inchesToMeters(22.75);

        public static final NEOCoaxialSwerveModule.SwerveConstants SWERVE_CONSTANTS = new NEOCoaxialSwerveModule.SwerveConstants();
        static {
            SWERVE_CONSTANTS.DRIVE_kP = 2.9646;
            SWERVE_CONSTANTS.DRIVE_kI = 0.0;
            SWERVE_CONSTANTS.DRIVE_kD = 0.0;
            SWERVE_CONSTANTS.DRIVE_kS = 0.0;
            SWERVE_CONSTANTS.DRIVE_kV = 2.8024;
            SWERVE_CONSTANTS.DRIVE_kA = 0.057223;
            SWERVE_CONSTANTS.STEER_kP = 6.0;
            SWERVE_CONSTANTS.STEER_kI = 0.0;
            SWERVE_CONSTANTS.STEER_kD = 0.1;

            SWERVE_CONSTANTS.DRIVE_GEARING = 6.75;
            SWERVE_CONSTANTS.STEER_GEARING = 150.0 / 7.0;
            SWERVE_CONSTANTS.WHEEL_CIRCUMFERENCE = 2.0 * Math.PI * 0.0478;
            SWERVE_CONSTANTS.DRIVE_ENCODER_ROTATIONS_TO_METERS = SWERVE_CONSTANTS.WHEEL_CIRCUMFERENCE
                    / SWERVE_CONSTANTS.DRIVE_GEARING;
            SWERVE_CONSTANTS.STEER_ENCODER_ROTATIONS_TO_RADIANS = 2 * Math.PI / SWERVE_CONSTANTS.STEER_GEARING;

            SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND = 4.282;
            SWERVE_CONSTANTS.MAX_DRIVING_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;
            SWERVE_CONSTANTS.MAX_STEER_VELOCITY_RADIANS_PER_SECOND = 8.829 * Math.PI;
            // max velocity in 1/3 sec
            SWERVE_CONSTANTS.MAX_STEER_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 8.829 * 3 * Math.PI;

            SWERVE_CONSTANTS.DRIVE_CURRENT_LIMIT = 50;
            SWERVE_CONSTANTS.STEER_CURRENT_LIMIT = 20;
            SWERVE_CONSTANTS.DRIVE_GEARBOX_REPR = DCMotor.getNEO(1);
            SWERVE_CONSTANTS.STEER_GEARBOX_REPR = DCMotor.getNEO(1);
            SWERVE_CONSTANTS.DRIVE_MOI = 0.04;
            SWERVE_CONSTANTS.STEER_MOI = 0.025;
        }

        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = Math.PI / 4;
        public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 2 * Math.PI;

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

        public static final double XY_kP = 1;
        public static final double XY_kI = 0;
        public static final double XY_kD = 0;
        public static final TrapezoidProfile.Constraints XY_CONSTRAINTS = new TrapezoidProfile.Constraints(
                SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND,
                SWERVE_CONSTANTS.MAX_DRIVING_ACCELERATION_METERS_PER_SECOND_SQUARED);

        public static final double THETA_kP = 1;
        public static final double THETA_kI = 0;
        public static final double THETA_kD = 0.1;
        public static final TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(
                20 * Math.PI, 20 * Math.PI);

        public static final double PATH_FOLLOWING_TRANSLATION_kP = 7;
        public static final double PATH_FOLLOWING_ROTATION_kP = 7;

        public static final double BALANCE_kP = 0.035;
        public static final double BALANCE_kI = 0;
        public static final double BALANCE_kD = 0.004;
        public static final double BALANCE_TOLERANCE = 3;
        public static final double BALANCE_MAX_VALUE = 0.6;
    }

    public static final class Elevator {
        public static enum ElevatorPosition {
            BOTTOM(0),
            CONE_LOW(0.25),
            CONE_MID(1.14),
            CONE_HIGH(1.7),
            CUBE_LOW(0.25),
            CUBE_MID(1.18482),
            CUBE_HIGH(1.65),
            SINGLE_SUBSTATION_PICKUP(0.5164),
            TOP(1.40);
            // 1.72 meters max

            public final double value;

            private ElevatorPosition(double value) {
                this.value = value;
            }
        }

        public static final int LEFT_MOTOR_ID = 9;
        public static final int RIGHT_MOTOR_ID = 10;

        public static final int BOTTOM_HALL_SENSOR_PORT = 0;

        public static final DCMotor MOTOR_GEARBOX_REPR = DCMotor.getNEO(2);
        public static final double GEARING = 4.0;
        public static final double CARRIAGE_MASS_KG = 5;
        public static final double DRUM_RADIUS_METERS = Units.inchesToMeters(1.2); // 0.0305
        public static final double WHEEL_CIRCUMFERENCE = 2.0 * Math.PI * DRUM_RADIUS_METERS;
        public static final double ENCODER_ROTATIONS_TO_METERS = WHEEL_CIRCUMFERENCE / GEARING;

        public static final double MIN_HEIGHT_METERS = 0.0;
        public static final double MAX_HEIGHT_METERS = 1.65;

        public static final double ANGLE = Units.degreesToRadians(90 - 32);

        public static final int CURRENT_LIMIT = 40;

        public static final double kP = 60;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 0.25696;
        public static final double kG = 0.20167;
        public static final double kV = 2.3833;
        public static final double kA = 0.29009;
        public static final double TOLERANCE = 0.015;

        public static final double MAX_VELOCITY_METERS_PER_SECOND = 4;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;
        public static final double MAX_STOWING_VELOCITY_METERS_PER_SECOND = 2;
        public static final double MAX_STOWING_ACCELERATION_METERS_PER_SECOND_SQUARED = 2;

        public static final TrapezoidProfile.Constraints NORMAL_MOVEMENT_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_VELOCITY_METERS_PER_SECOND,
                MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
        public static final TrapezoidProfile.Constraints STOWING_MOVEMENT_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_STOWING_VELOCITY_METERS_PER_SECOND,
                MAX_STOWING_ACCELERATION_METERS_PER_SECOND_SQUARED);

        public static final Pose3d BASE_CARRIAGE_POSE = new Pose3d(-0.18, 0, 0.30, new Rotation3d(0, ANGLE, 0));
        public static final Pose3d BASE_INNER_SEGMENT_POSE = new Pose3d(-0.245, 0, 0.242, new Rotation3d());
        public static final Pose3d BASE_MIDDLE_SEGMENT_POSE = new Pose3d(-0.285, 0, 0.22, new Rotation3d());

        public static final double STARTING_INNER_SEGMENT_POSITION = 0.512;
        public static final double STARTING_MIDDLE_SEGMENT_POSITION = 1.075;
    }

    public static final class Elbow {
        public static enum ElbowPosition {
            LOW(-0.31),
            MID_STOW(-0.1),
            MID(-0.05),
            MID_CONE_HIGH(0.3),
            CONE_PICKUP(0.07),
            SINGLE_SUBSTATION_PICKUP(0.087),
            HIGH(0.75);

            public final double value;

            private ElbowPosition(double value) {
                this.value = value;
            }
        }

        public static final int MOTOR_ID = 11;

        public static final DCMotor MOTOR_GEARBOX_REPR = DCMotor.getNeo550(1);
        public static final double GEARING = 100.0;
        public static final double LENGTH_METERS = 0.3;
        public static final double MASS_KG = 3.63;
        public static final double MOMENT_OF_INERTIA_KG_METERS_SQUARED = SingleJointedArmSim.estimateMOI(LENGTH_METERS,
                MASS_KG);

        public static final double MIN_ANGLE_RADIANS = -Math.PI / 2.0;
        public static final double MAX_ANGLE_RADIANS = 0.267;

        public static final double ENCODER_ROTATIONS_TO_RADIANS = 2 * Math.PI / GEARING;
        public static final double ABSOLUTE_ENCODER_ROTATIONS_TO_RADIANS = 2 * Math.PI / GEARING;
        public static final double ABSOLUTE_ENCODER_ZERO_OFFSET = 1.274;

        public static final int CURRENT_LIMIT = 20;

        public static final double kP = 8;
        public static final double kI = 0;
        public static final double kD = 0.5;
        public static final double kS = 0.33055;
        public static final double kG = 0.54418;
        public static final double kV = 0.74458;
        public static final double kA = 0.044339;
        public static final double TOLERANCE = 0.05;

        public static final double MAX_VELOCITY_METERS_PER_SECOND = 18 * Math.PI;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 10 * Math.PI;
        public static final TrapezoidProfile.Constraints MOVEMENT_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_VELOCITY_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

        public static final Pose3d BASE_COMPONENT_POSE = new Pose3d(-0.1435, 0, 0.245, new Rotation3d());
        public static final Translation3d ELEVATOR_TO_ELBOW = new Translation3d(0.0365, 0, -0.055);
    }

    public static final class Manipulator {
        public static final int[] PINCERS_PORTS = { 1, 14 };
        public static final int[] WRIST_PORTS = { 2, 13 };
        public static final int POLE_SWITCH_PORT = 1;

        public static final Transform3d ELBOW_TO_WRIST = new Transform3d(new Translation3d(0.1425, 0, 0),
                new Rotation3d());
        // public static final Pose3d BASE_WRIST_POSE = new Pose3d(-0.01, 0, 0.245, new
        // Rotation3d());

        public static final Transform3d WRIST_TO_LEFT_PINCER = new Transform3d(new Translation3d(0.049, 0.10, -0.045),
                new Rotation3d());
        public static final Transform3d WRIST_TO_RIGHT_PINCER = new Transform3d(new Translation3d(0.066, -0.10, -0.045),
                new Rotation3d());
        // public static final Pose3d BASE_LEFT_PINCER_POSE = new Pose3d(0.039, 0.15,
        // 0.2, new Rotation3d());
        // public static final Pose3d BASE_RIGHT_PINCER_POSE = new Pose3d(0.056, -0.15,
        // 0.2, new Rotation3d());

        public static final double PINCER_MOVEMENT_SECTION = 0.07;

        public static final double WRIST_MOVEMENT_TIME = 0.3;
        public static final double PINCER_MOVEMENT_TIME = 0.2;
    }

    public static final class Intake {
        public static final int PASSOVER_LEFT_MOTOR_ID = 12;
        public static final int PASSOVER_RIGHT_MOTOR_ID = 13;

        public static final int[] PASSOVER_PORTS = { 3, 12 };
        public static final int[] CUBAPULT_PORTS = { 0, 15 };

        public static final int PASSOVER_CURRENT_LIMIT = 20;

        public static final Pose3d LEFT_PASSOVER_RETRACTED_POSE = new Pose3d(0.139, 0.253, 0.168, new Rotation3d());
        public static final Pose3d LEFT_PASSOVER_EXTENDED_POSE = new Pose3d(0.195, 0.17, 0.168, new Rotation3d());
        public static final Pose3d RIGHT_PASSOVER_RETRACTED_POSE = new Pose3d(0.139, -0.253, 0.168,
                new Rotation3d(0, 0, Math.PI));
        public static final Pose3d RIGHT_PASSOVER_EXTENDED_POSE = new Pose3d(0.195, -0.17, 0.168,
                new Rotation3d(0, 0, Math.PI));

        public static final double PASSOVER_MOVEMENT_TIME = 0.5;
    }

    public static final class LEDs {
        public static final int PORT = 0;
        public static final int LENGTH = 50;
    }

    public static final class Auto {
        public static final double TRAJECTORY_X_kP = 5;
        public static final double TRAJECTORY_Y_kP = 5;
        public static final double TRAJECTORY_THETA_kP = 1;
    }

    public static final class Teleop {
        /**
         * A value inputted into the rate limiter (the joystick input) can move from 0
         * to 1 in 1/RATE_LIMIT seconds.
         * 
         * A rate limit of 3, for example, means that 0->1 in 1/3 sec.
         * Larger numbers mean less of a rate limit.
         */
        public static final double JOYSTICK_INPUT_RATE_LIMIT = 15.0;
        public static final double JOYSTICK_CURVE_EXP = 2;
        public static final TunableDouble INPUT_LIMIT = new TunableDouble("Drivetrain", "Input Limit", 1);
    }

    public static final class Vision {
        public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1);

        public static final PhotonCameraConstants CAMERA_CONSTANTS = new PhotonCameraConstants();
        static {
            // CAMERA_CONSTANTS.WIDTH = 1280;
            // CAMERA_CONSTANTS.HEIGHT = 800;
            // CAMERA_CONSTANTS.FOV = 95.39;
            // CAMERA_CONSTANTS.FPS = 20;
            // CAMERA_CONSTANTS.AVG_LATENCY = 50;
            // CAMERA_CONSTANTS.STDDEV_LATENCY = 15;
            CAMERA_CONSTANTS.WIDTH = 1600;
            CAMERA_CONSTANTS.HEIGHT = 1200;
            CAMERA_CONSTANTS.FOV = 95.39;
            CAMERA_CONSTANTS.FPS = 30;
            CAMERA_CONSTANTS.AVG_LATENCY = 30;
            CAMERA_CONSTANTS.STDDEV_LATENCY = 15;
        }

        // front-left, front-right, back-left, back-right
        public static final Transform3d[] ROBOT_TO_CAMS = new Transform3d[] {
                new Transform3d(
                        new Translation3d(Units.inchesToMeters(9.5), Units.inchesToMeters(9.5),
                                Units.inchesToMeters(10)),
                        new Rotation3d(0, -Units.degreesToRadians(10), -Math.PI / 8)),
                new Transform3d(
                        new Translation3d(Units.inchesToMeters(-12.5), Units.inchesToMeters(0),
                                Units.inchesToMeters(10)),
                        new Rotation3d(0, -Units.degreesToRadians(10), Math.PI)),
                new Transform3d(
                        new Translation3d(Units.inchesToMeters(9.5), Units.inchesToMeters(-9.5),
                                Units.inchesToMeters(10)),
                        new Rotation3d(0, -Units.degreesToRadians(10), Math.PI / 8))
        };
        // public static final Transform3d[] ROBOT_TO_CAMS = new Transform3d[] {
        // new Transform3d(
        // new Translation3d(Units.inchesToMeters(-10.1376),
        // Units.inchesToMeters(-5.3876),
        // Units.inchesToMeters(28.826)),
        // new Rotation3d(0, 0, Math.PI / 4.0)),
        // new Transform3d(
        // new Translation3d(Units.inchesToMeters(-10.1376),
        // Units.inchesToMeters(-7.8624),
        // Units.inchesToMeters(28.826)),
        // new Rotation3d(0, 0, -Math.PI / 4.0)),

        // new Transform3d(
        // new Translation3d(Units.inchesToMeters(-12.6124),
        // Units.inchesToMeters(-5.3876),
        // Units.inchesToMeters(28.826)),
        // new Rotation3d(0, 0, 3.0 * Math.PI / 4.0)),
        // new Transform3d(
        // new Translation3d(Units.inchesToMeters(-12.6124),
        // Units.inchesToMeters(-7.8624),
        // Units.inchesToMeters(28.826)),
        // new Rotation3d(0, 0, -3.0 * Math.PI / 4.0)),
        // };
    }

    public static final class Mechanisms {
        public static final MechanismRoot2d ROOT = RobotContainer.mechanisms.getRoot("root", 2.5, 0.25);

        public static final MechanismLigament2d FROM_ROBOT = ROOT
                .append(new MechanismLigament2d("fromRobot", -0.33, 0, 0, new Color8Bit(Color.kWhite)));
        public static final MechanismLigament2d ELEVATOR_BASE_LIGAMENT = FROM_ROBOT
                .append(new MechanismLigament2d("elevatorBase", 0.71, 34, 4, new Color8Bit(Color.kCyan)));
        public static final MechanismLigament2d ELEVATOR_LIGAMENT = ELEVATOR_BASE_LIGAMENT
                .append(new MechanismLigament2d("elevator", 1.32, 0, 5, new Color8Bit(Color.kOrange)));
        public static final MechanismLigament2d ELBOW_LIGAMENT = ELEVATOR_LIGAMENT
                .append(new MechanismLigament2d("elbow", 0.2, -34, 3, new Color8Bit(Color.kGreen)));
        public static final MechanismLigament2d WRIST_LIGAMENT = ELBOW_LIGAMENT
                .append(new MechanismLigament2d("wrist", 0.2, 90, 3, new Color8Bit(Color.kRed)));

        public static final MechanismLigament2d TO_CUBAPULT = FROM_ROBOT
                .append(new MechanismLigament2d("toCubapult", 0.3, 90, 1, new Color8Bit(Color.kWhite)));
        public static final MechanismLigament2d CUBAPULT_LIGAMENT = TO_CUBAPULT
                .append(new MechanismLigament2d("cubapult", 0.3, 0, 3, new Color8Bit(Color.kPurple)));

        public static final MechanismLigament2d CUBAPOLT_EXT_LIGAMENT = CUBAPULT_LIGAMENT
                .append(new MechanismLigament2d("cubapultExt", 0.07, 90, 3, new Color8Bit(Color.kPurple)));
    }
}
// 2.298