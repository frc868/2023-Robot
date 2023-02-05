package frc.robot;

import java.util.Map;

import com.techhounds.houndutil.houndauto.AutoManager;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldConstants {
    public static double LENGTH_METERS = Units.inchesToMeters(651.25);
    public static double WIDTH_METERS = Units.inchesToMeters(315.5);

    public static class Blue {
        public static class LeftGrid {
            public static final Pose2d CONE_1 = new Pose2d(
                    new Translation2d(
                            Units.inchesToMeters(71.425),
                            Units.inchesToMeters(20)),
                    Rotation2d.fromDegrees(180));
            public static final Pose2d CUBE_2 = new Pose2d(
                    new Translation2d(
                            Units.inchesToMeters(71.425),
                            Units.inchesToMeters(42.125)),
                    Rotation2d.fromDegrees(180));
            public static final Pose2d CONE_3 = new Pose2d(
                    new Translation2d(
                            Units.inchesToMeters(71.425),
                            Units.inchesToMeters(64)),
                    Rotation2d.fromDegrees(180));
        }

        public static class MiddleGrid {
            public static final Pose2d CONE_4 = new Pose2d(
                    new Translation2d(
                            Units.inchesToMeters(71.425),
                            Units.inchesToMeters(86)),
                    Rotation2d.fromDegrees(180));
            public static final Pose2d CUBE_5 = new Pose2d(
                    new Translation2d(
                            Units.inchesToMeters(71.425),
                            Units.inchesToMeters(108.125)),
                    Rotation2d.fromDegrees(180));
            public static final Pose2d CONE_6 = new Pose2d(
                    new Translation2d(
                            Units.inchesToMeters(71.425),
                            Units.inchesToMeters(130)),
                    Rotation2d.fromDegrees(180));
        }

        public static class RightGrid {
            public static final Pose2d CONE_7 = new Pose2d(
                    new Translation2d(
                            Units.inchesToMeters(71.425),
                            Units.inchesToMeters(152)),
                    Rotation2d.fromDegrees(180));
            public static final Pose2d CUBE_8 = new Pose2d(
                    new Translation2d(
                            Units.inchesToMeters(71.425),
                            Units.inchesToMeters(174.125)),
                    Rotation2d.fromDegrees(180));
            public static final Pose2d CONE_9 = new Pose2d(
                    new Translation2d(
                            Units.inchesToMeters(71.425),
                            Units.inchesToMeters(196)),
                    Rotation2d.fromDegrees(180));
        }

        public static class GamePieces {
            public static final Pose2d ITEM_1 = new Pose2d(
                    new Translation2d(
                            Units.inchesToMeters(278.05),
                            Units.inchesToMeters(180.19)),
                    new Rotation2d());
            public static final Pose2d ITEM_2 = new Pose2d(
                    new Translation2d(
                            Units.inchesToMeters(278.05),
                            Units.inchesToMeters(132.19)),
                    new Rotation2d());
            public static final Pose2d ITEM_3 = new Pose2d(
                    new Translation2d(
                            Units.inchesToMeters(278.05),
                            Units.inchesToMeters(84.19)),
                    new Rotation2d());
            public static final Pose2d ITEM_4 = new Pose2d(
                    new Translation2d(
                            Units.inchesToMeters(278.05),
                            Units.inchesToMeters(36.19)),
                    new Rotation2d());
        }

        public static class ChargeStation {
            public static final double LENGTH_METERS = Units.inchesToMeters(97.25);
            public static final double WIDTH_METERS = Units.inchesToMeters(48.5);
            public static final Pose2d CENTER = new Pose2d(
                    new Translation2d(Units.inchesToMeters(150.8), Units.inchesToMeters(108.015)), new Rotation2d());
            public static final Pose2d UPPER_LEFT = CENTER
                    .plus(new Transform2d(new Translation2d(-WIDTH_METERS / 2, LENGTH_METERS / 2), new Rotation2d()));
            public static final Pose2d UPPER_RIGHT = CENTER
                    .plus(new Transform2d(new Translation2d(WIDTH_METERS / 2, LENGTH_METERS / 2), new Rotation2d()));
            public static final Pose2d LOWER_LEFT = CENTER
                    .plus(new Transform2d(new Translation2d(-WIDTH_METERS / 2, -LENGTH_METERS / 2), new Rotation2d()));
            public static final Pose2d LOWER_RIGHT = CENTER
                    .plus(new Transform2d(new Translation2d(WIDTH_METERS / 2, -LENGTH_METERS / 2), new Rotation2d()));
        }

        public static class Substations {
            public static final Pose2d DOUBLE_SUBSTATION_N = new Pose2d(
                    new Translation2d(
                            Units.inchesToMeters(14.275) + Constants.ROBOT_SIDE_LENGTH / 2,
                            Units.inchesToMeters(282.97 + 7)),
                    Rotation2d.fromDegrees(180));
            public static final Pose2d DOUBLE_SUBSTATION_S = new Pose2d(
                    new Translation2d(
                            Units.inchesToMeters(14.275) + Constants.ROBOT_SIDE_LENGTH / 2,
                            Units.inchesToMeters(248.2 - 7)),
                    Rotation2d.fromDegrees(180));

            public static final Pose2d SINGLE_SUBSTATION = new Pose2d(
                    new Translation2d(
                            Units.inchesToMeters(78.145),
                            WIDTH_METERS - Constants.ROBOT_SIDE_LENGTH),
                    Rotation2d.fromDegrees(90));

        }
    }

    public static class Red {
        public static class LeftGrid {
            public static final Pose2d CONE_1 = reflectBlueToRed(Blue.LeftGrid.CONE_1);
            public static final Pose2d CUBE_2 = reflectBlueToRed(Blue.LeftGrid.CUBE_2);
            public static final Pose2d CONE_3 = reflectBlueToRed(Blue.LeftGrid.CONE_3);
        }

        public static class MiddleGrid {
            public static final Pose2d CONE_4 = reflectBlueToRed(Blue.MiddleGrid.CONE_4);
            public static final Pose2d CUBE_5 = reflectBlueToRed(Blue.MiddleGrid.CUBE_5);
            public static final Pose2d CONE_6 = reflectBlueToRed(Blue.MiddleGrid.CONE_6);
        }

        public static class RightGrid {
            public static final Pose2d CONE_7 = reflectBlueToRed(Blue.RightGrid.CONE_7);
            public static final Pose2d CUBE_8 = reflectBlueToRed(Blue.RightGrid.CUBE_8);
            public static final Pose2d CONE_9 = reflectBlueToRed(Blue.RightGrid.CONE_9);
        }

        public static class GamePieces {
            public static final Pose2d ITEM_1 = reflectBlueToRed(Blue.GamePieces.ITEM_1);
            public static final Pose2d ITEM_2 = reflectBlueToRed(Blue.GamePieces.ITEM_2);
            public static final Pose2d ITEM_3 = reflectBlueToRed(Blue.GamePieces.ITEM_3);
            public static final Pose2d ITEM_4 = reflectBlueToRed(Blue.GamePieces.ITEM_4);
        }

        public static class ChargeStation {
            public static final double LENGTH_METERS = Units.inchesToMeters(97.25);
            public static final double WIDTH_METERS = Units.inchesToMeters(48.5);
            public static final Pose2d CENTER = reflectBlueToRed(Blue.ChargeStation.CENTER);
            public static final Pose2d UPPER_LEFT = reflectBlueToRed(Blue.ChargeStation.UPPER_LEFT);
            public static final Pose2d UPPER_RIGHT = reflectBlueToRed(Blue.ChargeStation.UPPER_RIGHT);
            public static final Pose2d LOWER_LEFT = reflectBlueToRed(Blue.ChargeStation.LOWER_LEFT);
            public static final Pose2d LOWER_RIGHT = reflectBlueToRed(Blue.ChargeStation.LOWER_RIGHT);
        }

        public static class Substations {
            public static final Pose2d DOUBLE_SUBSTATION_N = reflectBlueToRed(Blue.Substations.DOUBLE_SUBSTATION_N);
            public static final Pose2d DOUBLE_SUBSTATION_S = reflectBlueToRed(Blue.Substations.DOUBLE_SUBSTATION_S);
            public static final Pose2d SINGLE_SUBSTATION = reflectBlueToRed(Blue.Substations.SINGLE_SUBSTATION);
        }
    }

    public static Pose2d reflectBlueToRed(Pose2d blue) {
        return new Pose2d(new Translation2d(LENGTH_METERS - Math.abs(blue.getX()), blue.getY()),
                blue.getRotation().getDegrees() % 180 != 0 ? blue.getRotation()
                        : blue.getRotation().plus(Rotation2d.fromDegrees(180)));
    }

    public static Map<Alliance, Map<GamePieceLocation, Pose2d>> scoringLocationMap = Map.of(
            Alliance.Blue,
            Map.ofEntries(
                    Map.entry(GamePieceLocation.A1, Blue.LeftGrid.CONE_1),
                    Map.entry(GamePieceLocation.A2, Blue.LeftGrid.CONE_1),
                    Map.entry(GamePieceLocation.A3, Blue.LeftGrid.CONE_1),
                    Map.entry(GamePieceLocation.B1, Blue.LeftGrid.CUBE_2),
                    Map.entry(GamePieceLocation.B2, Blue.LeftGrid.CUBE_2),
                    Map.entry(GamePieceLocation.B3, Blue.LeftGrid.CUBE_2),
                    Map.entry(GamePieceLocation.C1, Blue.LeftGrid.CONE_3),
                    Map.entry(GamePieceLocation.C2, Blue.LeftGrid.CONE_3),
                    Map.entry(GamePieceLocation.C3, Blue.LeftGrid.CONE_3),
                    Map.entry(GamePieceLocation.D1, Blue.MiddleGrid.CONE_4),
                    Map.entry(GamePieceLocation.D2, Blue.MiddleGrid.CONE_4),
                    Map.entry(GamePieceLocation.D3, Blue.MiddleGrid.CONE_4),
                    Map.entry(GamePieceLocation.E1, Blue.MiddleGrid.CUBE_5),
                    Map.entry(GamePieceLocation.E2, Blue.MiddleGrid.CUBE_5),
                    Map.entry(GamePieceLocation.E3, Blue.MiddleGrid.CUBE_5),
                    Map.entry(GamePieceLocation.F1, Blue.MiddleGrid.CONE_6),
                    Map.entry(GamePieceLocation.F2, Blue.MiddleGrid.CONE_6),
                    Map.entry(GamePieceLocation.F3, Blue.MiddleGrid.CONE_6),
                    Map.entry(GamePieceLocation.G1, Blue.RightGrid.CONE_7),
                    Map.entry(GamePieceLocation.G2, Blue.RightGrid.CONE_7),
                    Map.entry(GamePieceLocation.G3, Blue.RightGrid.CONE_7),
                    Map.entry(GamePieceLocation.H1, Blue.RightGrid.CUBE_8),
                    Map.entry(GamePieceLocation.H2, Blue.RightGrid.CUBE_8),
                    Map.entry(GamePieceLocation.H3, Blue.RightGrid.CUBE_8),
                    Map.entry(GamePieceLocation.I1, Blue.RightGrid.CONE_9),
                    Map.entry(GamePieceLocation.I2, Blue.RightGrid.CONE_9),
                    Map.entry(GamePieceLocation.I3, Blue.RightGrid.CONE_9)),
            Alliance.Red,
            Map.ofEntries(
                    Map.entry(GamePieceLocation.A1, Red.LeftGrid.CONE_1),
                    Map.entry(GamePieceLocation.A2, Red.LeftGrid.CONE_1),
                    Map.entry(GamePieceLocation.A3, Red.LeftGrid.CONE_1),
                    Map.entry(GamePieceLocation.B1, Red.LeftGrid.CUBE_2),
                    Map.entry(GamePieceLocation.B2, Red.LeftGrid.CUBE_2),
                    Map.entry(GamePieceLocation.B3, Red.LeftGrid.CUBE_2),
                    Map.entry(GamePieceLocation.C1, Red.LeftGrid.CONE_3),
                    Map.entry(GamePieceLocation.C2, Red.LeftGrid.CONE_3),
                    Map.entry(GamePieceLocation.C3, Red.LeftGrid.CONE_3),
                    Map.entry(GamePieceLocation.D1, Red.MiddleGrid.CONE_4),
                    Map.entry(GamePieceLocation.D2, Red.MiddleGrid.CONE_4),
                    Map.entry(GamePieceLocation.D3, Red.MiddleGrid.CONE_4),
                    Map.entry(GamePieceLocation.E1, Red.MiddleGrid.CUBE_5),
                    Map.entry(GamePieceLocation.E2, Red.MiddleGrid.CUBE_5),
                    Map.entry(GamePieceLocation.E3, Red.MiddleGrid.CUBE_5),
                    Map.entry(GamePieceLocation.F1, Red.MiddleGrid.CONE_6),
                    Map.entry(GamePieceLocation.F2, Red.MiddleGrid.CONE_6),
                    Map.entry(GamePieceLocation.F3, Red.MiddleGrid.CONE_6),
                    Map.entry(GamePieceLocation.G1, Red.RightGrid.CONE_7),
                    Map.entry(GamePieceLocation.G2, Red.RightGrid.CONE_7),
                    Map.entry(GamePieceLocation.G3, Red.RightGrid.CONE_7),
                    Map.entry(GamePieceLocation.H1, Red.RightGrid.CUBE_8),
                    Map.entry(GamePieceLocation.H2, Red.RightGrid.CUBE_8),
                    Map.entry(GamePieceLocation.H3, Red.RightGrid.CUBE_8),
                    Map.entry(GamePieceLocation.I1, Red.RightGrid.CONE_9),
                    Map.entry(GamePieceLocation.I2, Red.RightGrid.CONE_9),
                    Map.entry(GamePieceLocation.I3, Red.RightGrid.CONE_9)));

    public static void displayOnField() {
        AutoManager.getInstance().getField().getObject("Blue_Cone_1").setPose(FieldConstants.Blue.LeftGrid.CONE_1);
        AutoManager.getInstance().getField().getObject("Blue_Cube_2").setPose(FieldConstants.Blue.LeftGrid.CUBE_2);
        AutoManager.getInstance().getField().getObject("Blue_Cone_3").setPose(FieldConstants.Blue.LeftGrid.CONE_3);
        AutoManager.getInstance().getField().getObject("Blue_Cone_4").setPose(FieldConstants.Blue.MiddleGrid.CONE_4);
        AutoManager.getInstance().getField().getObject("Blue_Cube_5").setPose(FieldConstants.Blue.MiddleGrid.CUBE_5);
        AutoManager.getInstance().getField().getObject("Blue_Cone_6").setPose(FieldConstants.Blue.MiddleGrid.CONE_6);
        AutoManager.getInstance().getField().getObject("Blue_Cone_7").setPose(FieldConstants.Blue.RightGrid.CONE_7);
        AutoManager.getInstance().getField().getObject("Blue_Cube_8").setPose(FieldConstants.Blue.RightGrid.CUBE_8);
        AutoManager.getInstance().getField().getObject("Blue_Cone_9").setPose(FieldConstants.Blue.RightGrid.CONE_9);
        AutoManager.getInstance().getField().getObject("Red_Cone_1").setPose(FieldConstants.Red.LeftGrid.CONE_1);
        AutoManager.getInstance().getField().getObject("Red_Cube_2").setPose(FieldConstants.Red.LeftGrid.CUBE_2);
        AutoManager.getInstance().getField().getObject("Red_Cone_3").setPose(FieldConstants.Red.LeftGrid.CONE_3);
        AutoManager.getInstance().getField().getObject("Red_Cone_4").setPose(FieldConstants.Red.MiddleGrid.CONE_4);
        AutoManager.getInstance().getField().getObject("Red_Cube_5").setPose(FieldConstants.Red.MiddleGrid.CUBE_5);
        AutoManager.getInstance().getField().getObject("Red_Cone_6").setPose(FieldConstants.Red.MiddleGrid.CONE_6);
        AutoManager.getInstance().getField().getObject("Red_Cone_7").setPose(FieldConstants.Red.RightGrid.CONE_7);
        AutoManager.getInstance().getField().getObject("Red_Cube_8").setPose(FieldConstants.Red.RightGrid.CUBE_8);
        AutoManager.getInstance().getField().getObject("Red_Cone_9").setPose(FieldConstants.Red.RightGrid.CONE_9);

        AutoManager.getInstance().getField().getObject("Blue_Set_Item_1")
                .setPose(FieldConstants.Blue.GamePieces.ITEM_1);
        AutoManager.getInstance().getField().getObject("Blue_Set_Item_2")
                .setPose(FieldConstants.Blue.GamePieces.ITEM_2);
        AutoManager.getInstance().getField().getObject("Blue_Set_Item_3")
                .setPose(FieldConstants.Blue.GamePieces.ITEM_3);
        AutoManager.getInstance().getField().getObject("Blue_Set_Item_4")
                .setPose(FieldConstants.Blue.GamePieces.ITEM_4);
        AutoManager.getInstance().getField().getObject("Red_Set_Item_1").setPose(FieldConstants.Red.GamePieces.ITEM_1);
        AutoManager.getInstance().getField().getObject("Red_Set_Item_2").setPose(FieldConstants.Red.GamePieces.ITEM_2);
        AutoManager.getInstance().getField().getObject("Red_Set_Item_3").setPose(FieldConstants.Red.GamePieces.ITEM_3);
        AutoManager.getInstance().getField().getObject("Red_Set_Item_4").setPose(FieldConstants.Red.GamePieces.ITEM_4);

        AutoManager.getInstance().getField().getObject("Blue_ChargeStation_UpperLeft")
                .setPose(FieldConstants.Blue.ChargeStation.UPPER_LEFT);
        AutoManager.getInstance().getField().getObject("Blue_ChargeStation_UpperRight")
                .setPose(FieldConstants.Blue.ChargeStation.UPPER_RIGHT);
        AutoManager.getInstance().getField().getObject("Blue_ChargeStation_LowerLeft")
                .setPose(FieldConstants.Blue.ChargeStation.LOWER_LEFT);
        AutoManager.getInstance().getField().getObject("Blue_ChargeStation_LowerRight")
                .setPose(FieldConstants.Blue.ChargeStation.LOWER_RIGHT);
        AutoManager.getInstance().getField().getObject("Blue_ChargeStation_Center")
                .setPose(FieldConstants.Blue.ChargeStation.CENTER);
        AutoManager.getInstance().getField().getObject("Red_ChargeStation_UpperLeft")
                .setPose(FieldConstants.Red.ChargeStation.UPPER_LEFT);
        AutoManager.getInstance().getField().getObject("Red_ChargeStation_UpperRight")
                .setPose(FieldConstants.Red.ChargeStation.UPPER_RIGHT);
        AutoManager.getInstance().getField().getObject("Red_ChargeStation_LowerLeft")
                .setPose(FieldConstants.Red.ChargeStation.LOWER_LEFT);
        AutoManager.getInstance().getField().getObject("Red_ChargeStation_LowerRight")
                .setPose(FieldConstants.Red.ChargeStation.LOWER_RIGHT);
        AutoManager.getInstance().getField().getObject("Red_ChargeStation_Center")
                .setPose(FieldConstants.Red.ChargeStation.CENTER);

        AutoManager.getInstance().getField().getObject("Blue_DoubleSubstation_N")
                .setPose(FieldConstants.Blue.Substations.DOUBLE_SUBSTATION_N);
        AutoManager.getInstance().getField().getObject("Blue_DoubleSubstation_S")
                .setPose(FieldConstants.Blue.Substations.DOUBLE_SUBSTATION_S);
        AutoManager.getInstance().getField().getObject("Blue_SingleSubstation")
                .setPose(FieldConstants.Blue.Substations.SINGLE_SUBSTATION);
        AutoManager.getInstance().getField().getObject("Red_DoubleSubstation_N")
                .setPose(FieldConstants.Red.Substations.DOUBLE_SUBSTATION_N);
        AutoManager.getInstance().getField().getObject("Red_DoubleSubstation_S")
                .setPose(FieldConstants.Red.Substations.DOUBLE_SUBSTATION_S);
        AutoManager.getInstance().getField().getObject("Red_SingleSubstation")
                .setPose(FieldConstants.Red.Substations.SINGLE_SUBSTATION);

    }
}