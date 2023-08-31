package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;
import java.util.function.Supplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;
import com.techhounds.houndutil.houndauto.AutoPath;
import com.techhounds.houndutil.houndauto.AutoRoutine;
import com.techhounds.houndutil.houndauto.AutoSetting;
import com.techhounds.houndutil.houndlib.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.AutoSettingValue;
import frc.robot.FieldConstants;
import frc.robot.GamePieceLocation;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Manipulator;

public class Autos {
    public static AutoRoutine northOnePiece(Drivetrain drivetrain, Intake intake,
            Manipulator manipulator, Elevator elevator, Elbow elbow) {
        List<AutoSetting> settings = List.of(
                new AutoSetting("Game Piece?",
                        new AutoSettingValue[] { AutoSettingValue.CONE, AutoSettingValue.CUBE,
                                AutoSettingValue.CUBAPULT }),
                new AutoSetting("Mobility?",
                        new AutoSettingValue[] { AutoSettingValue.YES, AutoSettingValue.NO }),
                new AutoSetting("Balance on Charge Station?",
                        new AutoSettingValue[] { AutoSettingValue.YES, AutoSettingValue.NO }));

        Supplier<List<PathPoint>> waypointsSupplier = () -> {
            List<PathPoint> waypoints = new ArrayList<PathPoint>();
            Pose2d pose;
            switch ((AutoSettingValue) settings.get(0).getValue()) {
                case CONE:
                    pose = FieldConstants.getStartingPoseFacingGrid(GamePieceLocation.I1);
                    waypoints.add(Utils.convertPose2dToPathPoint(
                            new Pose2d(pose.getTranslation(),
                                    Rotation2d.fromDegrees(0)),
                            pose.getRotation()));
                    break;
                case CUBE:
                    pose = FieldConstants.getStartingPoseFacingGrid(GamePieceLocation.H1);
                    waypoints.add(Utils.convertPose2dToPathPoint(
                            new Pose2d(pose.getTranslation(),
                                    Rotation2d.fromDegrees(0)),
                            pose.getRotation()));
                    break;
                case CUBAPULT:
                    waypoints.add(Utils.convertPose2dToPathPoint(
                            FieldConstants.getStartingCubapultPose(GamePieceLocation.H1), true));
                    break;
                default:
                    throw new IllegalArgumentException("Unexpected setting value!");
            }

            if (settings.get(1).getValue() == AutoSettingValue.YES) {
                switch ((AutoSettingValue) settings.get(0).getValue()) {
                    case CUBAPULT:
                        waypoints.add(
                                new PathPoint(FieldConstants.Blue.AutoPathPoints.North.MOBILITY,
                                        Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)));
                        break;
                    case CONE:
                    case CUBE:
                        waypoints.add(new PathPoint(FieldConstants.Blue.AutoPathPoints.North.MOBILITY,
                                Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180)));
                        break; // flip 180 if not using cubapult
                    default:
                        throw new IllegalArgumentException("Unexpected setting value!");

                }
            }
            if (settings.get(2).getValue() == AutoSettingValue.YES) {
                if (settings.get(1).getValue() == AutoSettingValue.YES) {
                    waypoints.add(
                            new PathPoint(
                                    FieldConstants.Blue.AutoPathPoints.North.CHARGE_STATION_BALANCE,
                                    Rotation2d.fromDegrees(180),
                                    Rotation2d.fromDegrees(
                                            settings.get(0).getValue() == AutoSettingValue.CUBAPULT ? 0 : 180))
                                    .withPrevControlLength(3));
                } else {
                    waypoints.add(new PathPoint(
                            FieldConstants.Blue.AutoPathPoints.North.CHARGE_STATION_BALANCE,
                            Rotation2d.fromDegrees(0),
                            Rotation2d.fromDegrees(settings.get(0).getValue() == AutoSettingValue.CUBAPULT ? 0 : 180))
                            .withPrevControlLength(3));
                }
            }
            return waypoints;
        };

        Supplier<AutoPath> autoPathSupplier = () -> {
            List<PathPoint> waypoints = waypointsSupplier.get();

            if (waypoints.size() == 1) {
                waypoints.add(waypoints.get(0)); // to make a trajectory nonetheless
            }
            return new AutoPath("North: 1 Piece",
                    PathPlanner.generatePath(new PathConstraints(4, 3), waypoints));
        };

        Supplier<Pose2d> blueInitialPoseSupplier = () -> {
            PathPoint startPoint = waypointsSupplier.get().get(0);
            return new Pose2d(startPoint.position, startPoint.holonomicRotation);
        };

        Function<AutoPath, CommandBase> command = (autoPath) -> Commands.sequence(
                Commands.either(
                        AutoCommands.launchCube(intake),
                        AutoCommands.scorePreload(
                                () -> settings.get(0).getValue() == AutoSettingValue.CONE
                                        ? GamePieceLocation.I1
                                        : GamePieceLocation.H1,
                                drivetrain, intake, manipulator, elevator, elbow),
                        () -> settings.get(0).getValue() == AutoSettingValue.CUBAPULT),
                AutoCommands.driveOut(autoPath.getTrajectories().get(0), drivetrain, intake, manipulator, elevator,
                        elbow));
        return new AutoRoutine("North: 1 Piece", settings, autoPathSupplier, blueInitialPoseSupplier, command);
    }

    public static AutoRoutine northTwoPiece(Drivetrain drivetrain, Intake intake,
            Manipulator manipulator, Elevator elevator, Elbow elbow) {
        List<AutoSetting> settings = List.of(
                new AutoSetting("Preload Game Piece?",
                        new AutoSettingValue[] { AutoSettingValue.CONE, AutoSettingValue.CUBE,
                                AutoSettingValue.CUBAPULT }),
                new AutoSetting("Second Game Piece?",
                        new AutoSettingValue[] { AutoSettingValue.CONE, AutoSettingValue.CUBE }),
                new AutoSetting("Second Game Piece Location?",
                        new AutoSettingValue[] { AutoSettingValue.ITEM1, AutoSettingValue.ITEM2 }),
                new AutoSetting("Balance on Charge Station?",
                        new AutoSettingValue[] { AutoSettingValue.YES, AutoSettingValue.NO }));

        Supplier<List<PathPoint>> waypointsSupplier = () -> {
            List<PathPoint> waypoints = new ArrayList<PathPoint>();
            Pose2d pose;
            switch ((AutoSettingValue) settings.get(0).getValue()) {
                case CONE:
                    pose = FieldConstants.getStartingPoseFacingGrid(GamePieceLocation.I1);
                    waypoints.add(Utils.convertPose2dToPathPoint(
                            new Pose2d(pose.getTranslation(),
                                    Rotation2d.fromDegrees(0)),
                            pose.getRotation()));
                    break;
                case CUBE:
                    pose = FieldConstants.getStartingPoseFacingGrid(GamePieceLocation.H1);
                    waypoints.add(Utils.convertPose2dToPathPoint(
                            new Pose2d(pose.getTranslation(),
                                    Rotation2d.fromDegrees(0)),
                            pose.getRotation()));
                    break;
                case CUBAPULT:
                    waypoints.add(Utils.convertPose2dToPathPoint(
                            FieldConstants.getStartingCubapultPose(GamePieceLocation.H1), true));
                    break;
                default:
                    throw new IllegalArgumentException("Unexpected setting value!");
            }

            if (settings.get(1).getValue() == AutoSettingValue.YES) {
                switch ((AutoSettingValue) settings.get(0).getValue()) {
                    case CUBAPULT:
                        waypoints.add(
                                new PathPoint(FieldConstants.Blue.AutoPathPoints.North.MOBILITY,
                                        Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)));
                        break;
                    case CONE:
                    case CUBE:
                        waypoints.add(new PathPoint(FieldConstants.Blue.AutoPathPoints.North.MOBILITY,
                                Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180)));
                        break; // flip 180 if not using cubapult
                    default:
                        throw new IllegalArgumentException("Unexpected setting value!");

                }
            }
            if (settings.get(2).getValue() == AutoSettingValue.YES) {
                if (settings.get(1).getValue() == AutoSettingValue.YES) {
                    waypoints.add(
                            new PathPoint(
                                    FieldConstants.Blue.AutoPathPoints.North.CHARGE_STATION_BALANCE,
                                    Rotation2d.fromDegrees(180),
                                    Rotation2d.fromDegrees(
                                            settings.get(0).getValue() == AutoSettingValue.CUBAPULT ? 0 : 180))
                                    .withPrevControlLength(3));
                } else {
                    waypoints.add(new PathPoint(
                            FieldConstants.Blue.AutoPathPoints.North.CHARGE_STATION_BALANCE,
                            Rotation2d.fromDegrees(0),
                            Rotation2d.fromDegrees(settings.get(0).getValue() == AutoSettingValue.CUBAPULT ? 0 : 180))
                            .withPrevControlLength(3));
                }
            }
            return waypoints;
        };

        Supplier<AutoPath> autoPathSupplier = () -> {
            List<PathPoint> waypoints = waypointsSupplier.get();

            if (waypoints.size() == 1) {
                waypoints.add(waypoints.get(0)); // to make a trajectory nonetheless
            }
            return new AutoPath("North: 1 Piece",
                    PathPlanner.generatePath(new PathConstraints(4, 3), waypoints));
        };

        Supplier<Pose2d> blueInitialPoseSupplier = () -> {
            PathPoint startPoint = waypointsSupplier.get().get(0);
            return new Pose2d(startPoint.position, startPoint.holonomicRotation);
        };

        Function<AutoPath, CommandBase> command = (autoPath) -> Commands.sequence(
                Commands.either(
                        AutoCommands.launchCube(intake),
                        AutoCommands.scorePreload(
                                () -> settings.get(0).getValue() == AutoSettingValue.CONE
                                        ? GamePieceLocation.I1
                                        : GamePieceLocation.H1,
                                drivetrain, intake, manipulator, elevator, elbow),
                        () -> settings.get(0).getValue() == AutoSettingValue.CUBAPULT),
                AutoCommands.driveOut(autoPath.getTrajectories().get(0), drivetrain, intake, manipulator, elevator,
                        elbow));
        return new AutoRoutine("North: 1 Piece", settings, autoPathSupplier, blueInitialPoseSupplier, command);
    }

    public static AutoRoutine southOnePiece(Drivetrain drivetrain, Intake intake,
            Manipulator manipulator, Elevator elevator, Elbow elbow) {
        List<AutoSetting> settings = List.of(
                new AutoSetting("Game Piece?",
                        new AutoSettingValue[] { AutoSettingValue.CONE, AutoSettingValue.CUBE,
                                AutoSettingValue.CUBAPULT }),
                new AutoSetting("Mobility?",
                        new AutoSettingValue[] { AutoSettingValue.YES, AutoSettingValue.NO }),
                new AutoSetting("Balance on Charge Station?",
                        new AutoSettingValue[] { AutoSettingValue.YES, AutoSettingValue.NO }));

        Supplier<List<PathPoint>> waypointsSupplier = () -> {
            List<PathPoint> waypoints = new ArrayList<PathPoint>();
            Pose2d pose;
            switch ((AutoSettingValue) settings.get(0).getValue()) {
                case CONE:
                    pose = FieldConstants.getStartingPoseFacingGrid(GamePieceLocation.A1);
                    waypoints.add(Utils.convertPose2dToPathPoint(
                            new Pose2d(pose.getTranslation(),
                                    Rotation2d.fromDegrees(0)),
                            pose.getRotation()));
                    break;
                case CUBE:
                    pose = FieldConstants.getStartingPoseFacingGrid(GamePieceLocation.B1);
                    waypoints.add(Utils.convertPose2dToPathPoint(
                            new Pose2d(pose.getTranslation(),
                                    Rotation2d.fromDegrees(0)),
                            pose.getRotation()));
                    break;
                case CUBAPULT:
                    waypoints.add(Utils.convertPose2dToPathPoint(
                            FieldConstants.getStartingCubapultPose(GamePieceLocation.B1), true));
                    break;
                default:
                    throw new IllegalArgumentException("Unexpected setting value!");
            }

            if (settings.get(1).getValue() == AutoSettingValue.YES) {
                switch ((AutoSettingValue) settings.get(0).getValue()) {
                    case CUBAPULT:
                        waypoints.add(
                                new PathPoint(FieldConstants.Blue.AutoPathPoints.South.MOBILITY,
                                        Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)));
                        break;
                    case CONE:
                    case CUBE:
                        waypoints.add(new PathPoint(FieldConstants.Blue.AutoPathPoints.South.MOBILITY,
                                Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180)));
                        break; // flip 180 if not using cubapult
                    default:
                        throw new IllegalArgumentException("Unexpected setting value!");

                }
            }
            if (settings.get(2).getValue() == AutoSettingValue.YES) {
                if (settings.get(1).getValue() == AutoSettingValue.YES) {
                    waypoints.add(
                            new PathPoint(
                                    FieldConstants.Blue.AutoPathPoints.South.CHARGE_STATION_BALANCE,
                                    Rotation2d.fromDegrees(180),
                                    Rotation2d.fromDegrees(
                                            settings.get(0).getValue() == AutoSettingValue.CUBAPULT ? 0 : 180))
                                    .withPrevControlLength(3));
                } else {
                    waypoints.add(new PathPoint(
                            FieldConstants.Blue.AutoPathPoints.South.CHARGE_STATION_BALANCE,
                            Rotation2d.fromDegrees(0),
                            Rotation2d.fromDegrees(settings.get(0).getValue() == AutoSettingValue.CUBAPULT ? 0 : 180))
                            .withPrevControlLength(3));
                }
            }
            return waypoints;
        };

        Supplier<AutoPath> autoPathSupplier = () -> {
            List<PathPoint> waypoints = waypointsSupplier.get();

            if (waypoints.size() == 1) {
                waypoints.add(waypoints.get(0)); // to make a trajectory nonetheless
            }
            return new AutoPath("South: 1 Piece",
                    PathPlanner.generatePath(new PathConstraints(4, 3), waypoints));
        };

        Supplier<Pose2d> blueInitialPoseSupplier = () -> {
            PathPoint startPoint = waypointsSupplier.get().get(0);
            return new Pose2d(startPoint.position, startPoint.holonomicRotation);
        };

        Function<AutoPath, CommandBase> command = (autoPath) -> Commands.sequence(
                Commands.either(
                        AutoCommands.launchCube(intake),
                        AutoCommands.scorePreload(
                                () -> settings.get(0).getValue() == AutoSettingValue.CONE
                                        ? GamePieceLocation.A1
                                        : GamePieceLocation.B1,
                                drivetrain, intake, manipulator, elevator, elbow),
                        () -> settings.get(0).getValue() == AutoSettingValue.CUBAPULT),
                AutoCommands.driveOut(autoPath.getTrajectories().get(0), drivetrain, intake, manipulator, elevator,
                        elbow));
        return new AutoRoutine("South: 1 Piece", settings, autoPathSupplier, blueInitialPoseSupplier, command);
    }
}
